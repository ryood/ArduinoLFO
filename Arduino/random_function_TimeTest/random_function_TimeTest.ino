/*
 * random()の処理時間を計測
 * 
 * 2018.01.29
 * 
 */

#define LOOP_N  (1000)

void setup() {
  unsigned long start, end, elapse;
  char strBuffer[80];
  
  Serial.begin(9600);
  Serial.println("\r\nrandom() Test\r\n");
  sprintf(strBuffer, "LOOP_N\t%d\r\n", LOOP_N);
  Serial.println(strBuffer);

  start = micros();
  for (int i = 0; i < LOOP_N; i++) {
      long r = random();
      //Serial.println(r);
  }
  end = micros();
  elapse = end - start;
  sprintf(strBuffer, "%ld\t%ld\t%ld\r\n", start, end, elapse);
  Serial.print(strBuffer);
}

void loop() {
}
