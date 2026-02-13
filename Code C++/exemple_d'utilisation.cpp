// exemple d'utilisation 
TS_Params p;
p.up   = {0.5023f, 0.9021f, 1.0109f, -3.2283f, -1.5653f};
p.down = {-0.1665f, 0.7224f, 0.7070f,  2.4590f,  0.9289f};
p.m = 2e-3f; p.p = 0.013f;
p.p1 = 3.9f; p.p0 = 0.4f; p.pdtheta = 0.012f;

TorqueSensorLib ts(fdc_dev_handle, p);
ts.begin(/*capdac=*/0, /*timeout_ms=*/50);

TS_Output o = ts.update(0.01f);
