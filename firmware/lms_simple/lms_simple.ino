// NLMS adaptive FIR identification of an RC plant (12 or 24 taps).
// D9 drives the RC network (PWM input); A1 reads the amplifier output d.
//
// Serial Monitor at 115200 baud:  g = train, t = test, a/b/c = hold duty 40/120/200, m = read output

const int PIN_PWM = 9;
const int PIN_D = A1;

const float VREF = 5.14;       // measured supply rail voltage (ADC reference)
const int T_MS = 200;          // sample interval in milliseconds
const int M = 12;              // number of filter taps: 12 or 24 (M x 0.2 s of input history)
const float MU = 0.2;          // learning step size
const int N_TRAIN = 1000;      // maximum number of training steps
const int WINDOW = 100;        // steps per RMS window
const float RMS_STOP = (M == 12) ? 0.020 : 0.015;  // window RMS stopping threshold (V): 20 mV for 12 taps, 15 mV for 24 taps
const int WINDOWS_TO_STOP = 3; // consecutive windows below the threshold required to stop
const int N_TEST = 200;        // test steps

float w[M];                    // filter weights: w[k] multiplies the input k samples ago
float u[M];                    // input history: u[0] is the newest (duty scaled to -1..+1)
float bias = 0;                // bias (constant output term)
float lastD = 0, lastY = 0;    // d and y from the most recent step

// Read the output d in volts (average of 16 readings).
float readD() {
  analogRead(PIN_D);                       // discard first conversion after channel select
  long sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(PIN_D);
  return (sum / 16.0) * VREF / 1024.0;
}

// One step: send a random input, wait, read d, compare with the prediction.
// If learn is true, also update the weights. Returns the error e = d - y.
float oneStep(bool learn) {
  int duty = random(40, 201);                         // random duty between 40 and 200
  analogWrite(PIN_PWM, duty);                         // apply the input to the plant

  for (int k = M - 1; k > 0; k--) u[k] = u[k - 1];    // shift the input history by one sample
  u[0] = (duty - 120) / 80.0;                         // insert the newest input

  delay(T_MS);                                        // hold the input for one sample interval
  float d = readD();                                  // measured plant output

  float y = bias;                                     // filter prediction y
  for (int k = 0; k < M; k++) y += w[k] * u[k];

  float e = d - y;                                    // prediction error
  lastD = d;  lastY = y;                              // stored for reporting

  if (learn) {
    float S = 1.0;                                    // 1 (for the bias) + sum of u squared
    for (int k = 0; k < M; k++) S += u[k] * u[k];
    float g = MU * e / (0.001 + S);                   // normalized step
    bias += g;                                        // bias update
    for (int k = 0; k < M; k++) w[k] += g * u[k];     // NLMS weight update
  }
  return e;
}

// Hold the mid-scale duty for 5 s so the capacitor settles, and clear the input history.
void settle() {
  for (int k = 0; k < M; k++) u[k] = 0;
  analogWrite(PIN_PWM, 120);
  delay(5000);
}

void showWeights() {
  float sum = 0;
  Serial.println(F("k, weight"));
  for (int k = 0; k < M; k++) {
    Serial.print(k);
    Serial.print(F(", "));
    Serial.println(w[k], 4);
    sum += w[k];
  }
  Serial.print(F("bias: "));
  Serial.println(bias, 4);
  Serial.print(F("sum of weights: "));
  Serial.println(sum, 4);
  if (w[0] > 0 && w[5] > 0 && w[5] < w[0]) {
    float a = pow(w[5] / w[0], 0.2);                  // per-sample decay ratio of the weights
    Serial.print(F("tau estimate (s): "));
    Serial.println(-(T_MS / 1000.0) / log(a), 3);
  }
}

void train() {
  bias = 0;
  for (int k = 0; k < M; k++) w[k] = 0;               // initialise weights to zero
  settle();
  Serial.println(F("step, d (V), y (V), e (mV), window RMS error (mV)"));
  float sum = 0;
  int below = 0;                                      // consecutive windows below the threshold
  int n;
  for (n = 1; n <= N_TRAIN; n++) {
    float e = oneStep(true);
    sum += e * e;
    if (n % WINDOW == 0) {                            // end of a window: report this step and the window RMS
      float rms = sqrt(sum / WINDOW);
      Serial.print(n);
      Serial.print(F(", "));
      Serial.print(lastD, 4);
      Serial.print(F(", "));
      Serial.print(lastY, 4);
      Serial.print(F(", "));
      Serial.print(e * 1000.0, 1);
      Serial.print(F(", "));
      Serial.println(rms * 1000.0, 1);
      sum = 0;
      if (rms < RMS_STOP) below++;
      else below = 0;
      if (below >= WINDOWS_TO_STOP) break;            // converged: stop training early
    }
  }
  if (n <= N_TRAIN) {
    Serial.print(F("Training stopped at step "));
    Serial.print(n);
    Serial.print(F(": window RMS below threshold for "));
    Serial.print(WINDOWS_TO_STOP);
    Serial.println(F(" consecutive windows."));
  } else {
    Serial.println(F("Training stopped: maximum step count reached."));
  }
  analogWrite(PIN_PWM, 120);
  showWeights();
}

void test() {
  settle();
  float sum = 0, worst = 0;
  for (int n = 0; n < N_TEST; n++) {
    float e = oneStep(false);                         // weights frozen during test
    sum += e * e;
    if (fabs(e) > worst) worst = fabs(e);
  }
  analogWrite(PIN_PWM, 120);
  Serial.print(F("TEST rms error (mV): "));
  Serial.println(sqrt(sum / N_TEST) * 1000.0, 1);
  Serial.print(F("TEST max error (mV): "));
  Serial.println(worst * 1000.0, 1);
  Serial.print(F("Last test step: d (V) = "));      // values from the final test step
  Serial.print(lastD, 4);
  Serial.print(F(", y (V) = "));
  Serial.print(lastY, 4);
  Serial.print(F(", e (mV) = "));
  Serial.println((lastD - lastY) * 1000.0, 1);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PWM, OUTPUT);
  analogWrite(PIN_PWM, 0);
  randomSeed(analogRead(A5));
  Serial.println(F("Ready. g = train, t = test, a/b/c = hold duty 40/120/200, m = read output"));
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'g') train();
  if (c == 't') test();
  if (c == 'a') { analogWrite(PIN_PWM, 40);  Serial.println(F("holding duty 40")); }
  if (c == 'b') { analogWrite(PIN_PWM, 120); Serial.println(F("holding duty 120")); }
  if (c == 'c') { analogWrite(PIN_PWM, 200); Serial.println(F("holding duty 200")); }
  if (c == 'm') { Serial.print(F("d = ")); Serial.println(readD(), 4); }   // single output reading
}
