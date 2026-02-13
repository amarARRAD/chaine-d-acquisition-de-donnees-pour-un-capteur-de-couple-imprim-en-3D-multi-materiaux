//header déclarations + structures

#pragma once
#include <cstdint>
#include <cmath>
#include "driver/i2c_master.h"


struct TS_HyperbolaParams {
  float a, b, c, x0, y0;
};

struct TS_Params {
  // Terme dynamique 
  float m = 2e-3f;
  float p = 0.013f;

  // Capa -> angle 
  float p1 = 3.9f;
  float p0 = 0.4f;
  float pdtheta = 0.012f;

  TS_HyperbolaParams up;
  TS_HyperbolaParams down;
};


// Sorties principales

struct TS_Output {
  float CIN1_pf = 0.f;
  float CIN2_pf = 0.f;
  float CIN3_pf = 0.f;
  float CIN4_pf = 0.f;

  // Mapping 
  // C1=CIN4, C2=CIN3, C3=CIN2, C4=CIN1
  float C1_pf = 0.f;
  float C2_pf = 0.f;
  float C3_pf = 0.f;
  float C4_pf = 0.f;

  float Ctot = 0.f;

  float theta_estim_deg = 0.f;

  float tau_stat = 0.f;
  float tau_dyn  = 0.f;
  float tau_tot  = 0.f;

  int direction = -1; // +1 branche montante, -1 branche descendante
};


// Librairie principale

// Cette classe encapsule:
// - Driver FDC1004 (I2C)
// - Traduction de torque_angle_estimator.m (hystérésis)
// - Pipeline de calcul (Ctot, angle, couple statique/dynamique)
class TorqueSensorLib {
public:
  // Adresse I2C FDC1004 
  static constexpr uint8_t FDC1004_ADDR = 0x50;

  // Constructeur: le handle device I2C est créé côté application ESP-IDF.
  TorqueSensorLib(i2c_master_dev_handle_t dev, const TS_Params& params);

  // Initialisation FDC1004 + configuration des 4 mesures CINx / CAPGND.
  // capdac: 0..31 (offset hardware).
  // timeout_ms: délai max d'attente data ready (poll).
  bool begin(uint8_t capdac = 0, uint32_t timeout_ms = 50);

  // Mise à jour complète:
  // - lit CIN1..CIN4
  // - calcule Ctot
  // - calcule theta_estim + tau_stat + tau_dyn + tau_tot
  // dt_s: période d'échantillonnage en secondes (ex: 0.01f)
  TS_Output update(float dt_s);

  // Reset de l'état interne (hystérésis + dérivée).
  void reset();

private:
  
  // Driver FDC1004 
  
  enum class FDC_Channel : uint8_t {
    CIN1   = 0,
    CIN2   = 1,
    CIN3   = 2,
    CIN4   = 3,
    CAPGND = 4
  };

  bool fdc_write16(uint8_t reg, uint16_t v);
  bool fdc_read16(uint8_t reg, uint16_t& v);

  bool fdc_config_measure(uint8_t meas_1_to_4, FDC_Channel cha, FDC_Channel chb, uint8_t capdac);
  bool fdc_trigger(uint8_t meas_1_to_4);
  bool fdc_wait_ready(uint8_t meas_1_to_4, uint32_t timeout_ms);
  bool fdc_read_raw24(uint8_t meas_1_to_4, int32_t& raw24);

  // Conversion datasheet ±15 pF:
  // pF = raw/2^19 + capdac*3.125
  static float raw24_to_pf(int32_t raw24, uint8_t capdac);

 // hystérésis
  
  static inline float sgn(float x) { return (x > 0.f) - (x < 0.f); }

  static float hyperbola(float x, const TS_HyperbolaParams& p);
  float fup(float x) const;
  float fdown(float x) const;

  void compute_major_intersections();

  float hysteresis_update(float angle_deg);

  
  // Pipeline calcul
  
  static float compute_Ctot(float C1, float C2, float C3, float C4);
  float compute_dtheta(float theta, float dt_s);

private:
  i2c_master_dev_handle_t dev_;
  TS_Params params_;

  // FDC config
  uint8_t capdac_ = 0;
  uint32_t timeout_ms_ = 50;

  // Major loop intersections (fup==fdown)
  float angle_m_min_ = -5.f;
  float angle_M_max_ =  5.f;
  float torque_m_min_ = 0.f;
  float torque_M_max_ = 0.f;

  // Translation boucle mineure
  float angle_trans_ = 0.f;
  float torque_trans_ = 0.f;

  // Etat hystérésis
  int   direction_ = -1;
  float seuil_rel_ = 0.1f;
  float seuil_abs_ = NAN;
  float bidouille_ = 50.f;
  bool  hyst_inited_ = false;

  // Dérivée angle
  float theta_prev_ = 0.f;
  bool  theta_inited_ = false;
};
