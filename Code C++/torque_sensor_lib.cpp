// l’implémentation
#include "torque_sensor_lib.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Registres FDC1004
static constexpr uint8_t REG_MEAS1_MSB   = 0x00; // MEAS1 MSB (puis LSB à 0x01, etc.)
static constexpr uint8_t REG_MEAS_CONFIG = 0x0C; // trigger + DONE bits
static constexpr uint8_t REG_FDC_CONF    = 0x0F; // config globale
static constexpr uint8_t REG_MEASx_CFG   = 0x08; // 0x08..0x0B

TorqueSensorLib::TorqueSensorLib(i2c_master_dev_handle_t dev, const TS_Params& params)
: dev_(dev), params_(params) {
  reset();
  compute_major_intersections();
}


// Reset

void TorqueSensorLib::reset() {
  angle_trans_ = 0.f;
  torque_trans_ = 0.f;

  direction_ = -1;
  seuil_rel_ = 0.1f;
  seuil_abs_ = NAN;
  bidouille_ = 50.f;
  hyst_inited_ = false;

  theta_prev_ = 0.f;
  theta_inited_ = false;
}


// I2C low level

bool TorqueSensorLib::fdc_write16(uint8_t reg, uint16_t v) {
  uint8_t buf[3] = { reg, uint8_t(v >> 8), uint8_t(v & 0xFF) };
  return i2c_master_transmit(dev_, buf, sizeof(buf), pdMS_TO_TICKS(50)) == ESP_OK;
}

bool TorqueSensorLib::fdc_read16(uint8_t reg, uint16_t& v) {
  uint8_t r = reg;
  uint8_t in[2] = {0, 0};
  if (i2c_master_transmit_receive(dev_, &r, 1, in, 2, pdMS_TO_TICKS(50)) != ESP_OK) return false;
  v = (uint16_t(in[0]) << 8) | uint16_t(in[1]);
  return true;
}


// Driver FDC1004 (complet)

bool TorqueSensorLib::fdc_config_measure(uint8_t meas, FDC_Channel cha, FDC_Channel chb, uint8_t capdac) {
  if (meas < 1 || meas > 4) return false;
  if (capdac > 31) capdac = 31;

  const uint8_t reg = uint8_t(REG_MEASx_CFG + (meas - 1));

  // MEASx_CONFIG:
  // bits 15..13 CHA, 12..10 CHB, 9..5 CAPDAC
  uint16_t v = 0;
  v |= (uint16_t(uint8_t(cha) & 0x07) << 13);
  v |= (uint16_t(uint8_t(chb) & 0x07) << 10);
  v |= (uint16_t(capdac & 0x1F) << 5);

  return fdc_write16(reg, v);
}

bool TorqueSensorLib::fdc_trigger(uint8_t meas) {
  if (meas < 1 || meas > 4) return false;

  // Start bits 0..3
  uint16_t v = 0;
  v |= (1u << (meas - 1));
  return fdc_write16(REG_MEAS_CONFIG, v);
}

bool TorqueSensorLib::fdc_wait_ready(uint8_t meas, uint32_t timeout_ms) {
  if (meas < 1 || meas > 4) return false;

  // DONE bits: DONE1=bit12, DONE2=bit13, DONE3=bit14, DONE4=bit15
  const uint16_t done_mask = uint16_t(1u << (11 + meas));

  TickType_t t_end = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
  while (xTaskGetTickCount() < t_end) {
    uint16_t st = 0;
    if (!fdc_read16(REG_MEAS_CONFIG, st)) return false;
    if (st & done_mask) return true;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return false;
}

bool TorqueSensorLib::fdc_read_raw24(uint8_t meas, int32_t& raw24) {
  if (meas < 1 || meas > 4) return false;

  const uint8_t base = uint8_t(REG_MEAS1_MSB + (meas - 1) * 2);
  uint16_t msb = 0, lsb = 0;
  if (!fdc_read16(base, msb)) return false;
  if (!fdc_read16(base + 1, lsb)) return false;

  // 24-bit signed: MSB(16) + top8(LSB)
  int32_t v24 = (int32_t(msb) << 8) | (int32_t(lsb) >> 8);

  // sign extend 24-bit
  if (v24 & 0x800000) v24 |= ~0xFFFFFF;
  raw24 = v24;
  return true;
}

float TorqueSensorLib::raw24_to_pf(int32_t raw24, uint8_t capdac) {
  // Datasheet ±15 pF : LSB = 1/2^19 pF
  const float pf = float(raw24) / 524288.0f;
  const float off = float(capdac) * 3.125f;
  return pf + off;
}


// begin() : init FDC1004

bool TorqueSensorLib::begin(uint8_t capdac, uint32_t timeout_ms) {
  capdac_ = (capdac > 31) ? 31 : capdac;
  timeout_ms_ = timeout_ms;

  // Configuration globale 
  if (!fdc_write16(REG_FDC_CONF, 0x0000)) return false;

  // MEAS1..4 = CIN1..CIN4 vs CAPGND
  if (!fdc_config_measure(1, FDC_Channel::CIN1, FDC_Channel::CAPGND, capdac_)) return false;
  if (!fdc_config_measure(2, FDC_Channel::CIN2, FDC_Channel::CAPGND, capdac_)) return false;
  if (!fdc_config_measure(3, FDC_Channel::CIN3, FDC_Channel::CAPGND, capdac_)) return false;
  if (!fdc_config_measure(4, FDC_Channel::CIN4, FDC_Channel::CAPGND, capdac_)) return false;

  reset();
  compute_major_intersections();
  return true;
}


// Maths hyperboles 

float TorqueSensorLib::hyperbola(float x, const TS_HyperbolaParams& p) {
  // y = a*(1 - sqrt(1 + (x-x0)^2/b^2)) + c*(x-x0) + y0
  const float dx = x - p.x0;
  const float bb = p.b * p.b;
  return p.a * (1.f - std::sqrt(1.f + (dx * dx) / bb)) + p.c * dx + p.y0;
}

float TorqueSensorLib::fup(float x) const   { return hyperbola(x, params_.up); }
float TorqueSensorLib::fdown(float x) const { return hyperbola(x, params_.down); }

void TorqueSensorLib::compute_major_intersections() {
 
  // On recherche les changements de signe de (fup - fdown) sur [-40, 40].
  const float xmin = -40.f, xmax = 40.f;
  const int N = 4000;

  float roots[8];
  int n = 0;

  float px = xmin;
  float pf = fup(px) - fdown(px);

  for (int i = 1; i <= N; ++i) {
    float x = xmin + (xmax - xmin) * (float(i) / float(N));
    float f = fup(x) - fdown(x);

    if (sgn(f) != sgn(pf)) {
      float a = px, b = x;
      float fa = pf;

      for (int it = 0; it < 30; ++it) {
        float m = 0.5f * (a + b);
        float fm = fup(m) - fdown(m);
        if (sgn(fm) == sgn(fa)) { a = m; fa = fm; }
        else { b = m; }
      }
      if (n < 8) roots[n++] = 0.5f * (a + b);
    }

    px = x;
    pf = f;
  }

  if (n >= 2) {
    angle_m_min_ = roots[0];
    angle_M_max_ = roots[n - 1];
  }

  torque_m_min_ = fdown(angle_m_min_);
  torque_M_max_ = fdown(angle_M_max_);
}


// Hystérésis 

float TorqueSensorLib::hysteresis_update(float angle_deg) {
  if (!hyst_inited_) {
    seuil_abs_ = angle_deg;
    hyst_inited_ = true;
  }

  float torque = 0.f;

  // Couple statique = min/max entre courbe translatée et majeure
  if (direction_ == +1) {
    const float t_cur = fup(angle_deg - angle_trans_) + torque_trans_;
    const float t_maj = fup(angle_deg);
    torque = (t_cur < t_maj) ? t_cur : t_maj;

    // Retour sur majeure => translation remise à zéro
    if (torque == t_maj) { angle_trans_ = 0.f; torque_trans_ = 0.f; }
  } else {
    const float t_cur = fdown(angle_deg - angle_trans_) + torque_trans_;
    const float t_maj = fdown(angle_deg);
    torque = (t_cur > t_maj) ? t_cur : t_maj;

    if (torque == t_maj) { angle_trans_ = 0.f; torque_trans_ = 0.f; }
  }

  // Détection d'extrema + recalage translation (dichotomie)
  if (direction_ == -1) {
    // seuil_abs = min(seuil_abs, angle + seuil_rel)
    seuil_abs_ = std::fmin(seuil_abs_, angle_deg + seuil_rel_);

    if (angle_deg > seuil_abs_) {
      // MIN détecté
      const float angle_m  = angle_deg - bidouille_ * seuil_rel_;
      const float Torque_m = fdown(angle_m - angle_trans_) + torque_trans_;

      // Version "test": l'ancien maximum est pris sur la majeure
      const float angle_M  = angle_M_max_;
      const float Torque_M = torque_M_max_;

      // Solve: fup(angle_M-x) - fup(angle_m-x) - Torque_M + Torque_m = 0
      float a = -20.f, b = 20.f;
      const float eps = 0.05f;

      float fa = fup(angle_M - a) - fup(angle_m - a) - Torque_M + Torque_m;
      for (int it = 0; it < 200 && (b - a) > eps; ++it) {
        float x  = 0.5f * (a + b);
        float fx = fup(angle_M - x) - fup(angle_m - x) - Torque_M + Torque_m;
        if (sgn(fx) == sgn(fa)) { a = x; fa = fx; }
        else { b = x; }
      }

      const float x_sym = 0.5f * (a + b);
      angle_trans_  = x_sym;
      torque_trans_ = Torque_m - fup(angle_m - x_sym);

      direction_ = +1;
      seuil_abs_ = angle_deg;
    }
  } else {
    // direction_ == +1
    seuil_abs_ = std::fmax(seuil_abs_, angle_deg - seuil_rel_);

    if (angle_deg < seuil_abs_) {
      // MAX détecté
      const float angle_M  = angle_deg + bidouille_ * seuil_rel_;
      const float Torque_M = fup(angle_M - angle_trans_) + torque_trans_;

      // Version "test": l'ancien minimum est pris sur la majeure
      const float angle_m  = angle_m_min_;
      const float Torque_m = torque_m_min_;

      // Solve: fdown(angle_M-x) - fdown(angle_m-x) - Torque_M + Torque_m = 0
      float a = -20.f, b = 20.f;
      const float eps = 0.05f;

      float fa = fdown(angle_M - a) - fdown(angle_m - a) - Torque_M + Torque_m;
      for (int it = 0; it < 200 && (b - a) > eps; ++it) {
        float x  = 0.5f * (a + b);
        float fx = fdown(angle_M - x) - fdown(angle_m - x) - Torque_M + Torque_m;
        if (sgn(fx) == sgn(fa)) { a = x; fa = fx; }
        else { b = x; }
      }

      const float x_sym = 0.5f * (a + b);
      angle_trans_  = x_sym;
      torque_trans_ = Torque_m - fdown(angle_m - x_sym);

      direction_ = -1;
      seuil_abs_ = angle_deg;
    }
  }

  return torque;
}


// Pipeline couple

float TorqueSensorLib::compute_Ctot(float C1, float C2, float C3, float C4) {
  // Formule MATLAB : (-C1 - C3 + C2 + C4)/1000
  return (-C1 - C3 + C2 + C4) / 1000.f;
}

float TorqueSensorLib::compute_dtheta(float theta, float dt_s) {
  if (!theta_inited_ || dt_s <= 0.f) {
    theta_prev_ = theta;
    theta_inited_ = true;
    return 0.f;
  }
  float v = (theta - theta_prev_) / dt_s;
  theta_prev_ = theta;
  return v;
}


// update() : lecture + calcul complet

TS_Output TorqueSensorLib::update(float dt_s) {
  TS_Output out{};

  // 1) Déclenchement des 4 mesures
  for (int m = 1; m <= 4; ++m) {
    if (!fdc_trigger(uint8_t(m))) return out;
  }

  // 2) Attente data ready
  for (int m = 1; m <= 4; ++m) {
    if (!fdc_wait_ready(uint8_t(m), timeout_ms_)) return out;
  }

  // 3) Lecture raw + conversion en pF
  float cin_pf[4] = {0,0,0,0};
  for (int m = 1; m <= 4; ++m) {
    int32_t raw = 0;
    if (!fdc_read_raw24(uint8_t(m), raw)) return out;
    cin_pf[m - 1] = raw24_to_pf(raw, capdac_);
  }

  out.CIN1_pf = cin_pf[0];
  out.CIN2_pf = cin_pf[1];
  out.CIN3_pf = cin_pf[2];
  out.CIN4_pf = cin_pf[3];

  // 4) Mapping a (C1..C4 )
  // - CIN1 <- Cin4
  // - CIN2 <- Cin3
  // - CIN3 <- Cin2
  // - CIN4 <- Cin1
  
  out.C1_pf = out.CIN4_pf;
  out.C2_pf = out.CIN3_pf;
  out.C3_pf = out.CIN2_pf;
  out.C4_pf = out.CIN1_pf;

  // 5) Capacité différentielle
  out.Ctot = compute_Ctot(out.C1_pf, out.C2_pf, out.C3_pf, out.C4_pf);

  // 6) Angle estimé (structure MATLAB)
  float theta_base = params_.p1 * out.Ctot + params_.p0;
  float theta_dot  = compute_dtheta(theta_base, dt_s);

  // La correction directionnelle utilise la direction courante avant update hystérésis.
  const int dir_before = direction_;
  out.theta_estim_deg = theta_base + float(dir_before) * params_.pdtheta * theta_dot;

  // 7) Couple statique par hystérésis (sur theta_estim_deg)
  out.tau_stat = hysteresis_update(out.theta_estim_deg);
  out.direction = direction_;

  // 8) Couple dynamique (équation MATLAB)
  out.tau_dyn = (params_.m * out.theta_estim_deg - float(out.direction) * params_.p) * std::fabs(theta_dot);

  // 9) Couple total
  out.tau_tot = out.tau_stat + out.tau_dyn;

  return out;
}
