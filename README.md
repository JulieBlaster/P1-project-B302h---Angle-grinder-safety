#include <LinearRegression.h>
#include <math.h>
LinearRegression lr; // Define Objects
float values[2];       // Define Variables
float y,big, small, walue,a, b, r,bb, bs,bMax, bMin, volume,t, dist, minDist,volumeNew,avagA, biggestBb, smallestBs, avagBm,xholder, yholder, zholder, lenholder; 
int x, xBig, xSmall,counterLen, minLenNr,POGRAMMODE,xvalue,yvalue,zvalue,button,timer;
float *Pbig, *Psmall, *Pwalue,*Pa, *Pb, *Pr,*Pbb, *Pbs,*PbigVolume,*PminDist,*PbiggestBb, *PsmallestBs,*Pxholder, *Pyholder, *Pzholder, *Plenholder;
int *Px, *PxBig, *PxSmall,*PminLenNr;
struct matrix {         //the struct form.____________________________________________________
  float vMiddle[3][30]; //middlefunktion
  float newVektor[3][4];
};
struct matrix vektor; //the real struckt with the 30 vectors.
int counter = 0;
float bigVolume = 0;
int xpin = 25;     //input leg x aksis
int ypin = 26;     //input leg y aksis
int zpin = 27;     //input leg z aksis
TaskHandle_t Task1, Task2;
SemaphoreHandle_t baton;
unsigned long myTime;
void buttonClikt()
{
  if (myTime - timer > 300  ) {
    timer = myTime;
    if (button == 0) {
      button = 1;
    }
    else {
      button = 0;
    }
  }
}
// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  baton = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(
    codeForTask1,
    "led1Task",
    1000,
    NULL,
    1,
    &Task1,
    1); //nul  !!!
  delay(500);  // needed to start-up task1
  xTaskCreatePinnedToCore(
    codeForTask2,
    "led2Task",
    1000,
    NULL,
    1,
    &Task2,
    0); //1 !!!
  Pbig = &big;
  Psmall = &small;
  Pwalue = &walue;
  small = 100, big = 0, walue = 0, x = 0;
  xBig = 0, xSmall = 0;
  PxBig = &xBig;
  PxSmall = &xSmall;
  Pa = &a, Pb = &b, Pr = &r; //de her adresser er lig Pa.....
  Pbb = &bb, Pbs = &bs;
  PbigVolume = &bigVolume;
  PminDist = &minDist;
  minDist = 100000;
  PminLenNr = &minLenNr;
  counterLen = 0;
  POGRAMMODE = 0;
  PbiggestBb = &biggestBb;
  PsmallestBs = &smallestBs;
  Pxholder = &xholder;
  Pyholder = &yholder;
  Pzholder = &zholder;
  Plenholder = &lenholder;
  pinMode(32, OUTPUT);
  pinMode(22, INPUT_PULLUP); //pin 12 gets set to input
  attachInterrupt(22, buttonClikt, FALLING);
  button = 0; //0=closed
  avagA = 0.00001;
  avagBm = 1.0;
  biggestBb = 2.0;
  smallestBs = 0.5;
}

void asignValues ()       //this funktion take the analog data and asignes it walues for later use.
{
  xvalue = analogRead(xpin);                    //xvalue gets asigned to what xpin is analog reating
  //                   -    +
  int x = map(xvalue, 2380, 1610, -100, 100);     //maps the extreme ends analog values from -100 to 100 and x gets asigendet to that walue
  float xg = (float)x / (-100.00);                        //converts the mapped value into acceleration in terms of "g" that gets asigend xg
  *Pxholder = xg;

  yvalue = analogRead(ypin);
  int y = map(yvalue, 2369, 1524, -100, 100);
  float yg = (float)y / (-100.00);
  *Pyholder = yg;

  zvalue = analogRead(zpin);
  int z = map(zvalue, 2461, 1640, -100, 100);
  float zg = (float)z / (-100.00);
  *Pzholder = zg;
}
void asignValuesVector()  //this funktion take the funktion asignValues pointers resultat and make the first vectors,
{ //to make, the point walue
  lenholder = sqrt(pow(xholder, 2) + pow(yholder, 2) + pow(zholder, 2)); //the len af the A=<x,y,z>^%T
}
void linarRegression() {        // this funktions purpus is to asign a,b,r values form, linary regressionin to the vMiddle vector.
  for (int i = 0; i < 10; i++) // sinse there is 1000 points messured, ther wil be createt 10 linary regressionins of 100 points each.
  {
    for (int i = 0; i < 100; i++)
    {
      asignValues ();
      asignValuesVector();
      y = lenholder;      // this random simulate the numbers ther acturly wil be used.
      lr.Data(y);              // the data gets ritton in to the libery linarRegression.
      x++;                     // for each y x counts 1 up. so you can numberise y.
      *Pwalue = y;             // whats Pwalue points as will be eaqul to y. that is to find the biggest y, ands its x, and the smallest
      ekstremCoefficients();   // finds bMax and bMin (biggest b form biggest y smallest b for smallest y.
    }
    bLinarRegression();        //calgulates the b in MAX(y=ax+b) and the b in MIN(y=ax+b.
    printLinarRegression();    //ther is one important line of code in that funcotion witch asigns walues
    lr.Reset();                // Reset the libary.
    small = 100, big = 0, walue = 0; x = 0;         //resets the thing
    vektor.vMiddle[0][i] = (*Pa);
    vektor.vMiddle[1][i] = (*Pb);       //b mittle
    vektor.vMiddle[2][i] = (*Pr);
    vektor.vMiddle[0][i + 10] = (*Pa);
    vektor.vMiddle[1][i + 10] = (*Pbb); // b BIG
    vektor.vMiddle[2][i + 10] = (*Pr);
    vektor.vMiddle[0][i + 20] = (*Pa);
    vektor.vMiddle[1][i + 20] = (*Pbs); //b Small
    vektor.vMiddle[2][i + 20] = (*Pr);
  }
}
void ekstremCoefficients() {    // calgulate the ekstreem walues of y. big and small
  if (walue >= big) {
    *Pbig = walue;
    *PxBig = x;
  }
  if (walue <= small) {
    *Psmall = walue;
    *PxSmall = x;
  }
}
void bLinarRegression() {       // and form thees ekstreams y this fincotion calgulates a b(big) and a b(samll)
  *Pbb = -(values[0]) * xBig + big;
  *Pbs = -(values[0]) * xSmall + small;
}
void printLinarRegression() {   // the to only importent part is that a,bm,r , and it asigns values gets asigned else it just prints shit
  lr.Parameters(values); // IMPORTENT LINE
  *Pa = values[0], *Pb = values[1], *Pr = lr.Correlation(); //  IMPORTENT LINE
}
void callAria() {            
  for (int a1 = 0; a1 < 30; a1++) {
    for (int b1 = 0; b1 < 30; b1++) {
      for (int c1 = 0; c1 < 30; c1++) {
        for (int d1 = 0; d1 < 30; d1++) {
          float dotPruduct;
          counter++;
          volume = (fabs(((vektor.vMiddle[0][a1] - vektor.vMiddle[0][b1]) *
                          ((vektor.vMiddle[1][b1] - vektor.vMiddle[1][d1]) * (vektor.vMiddle[2][c1] - vektor.vMiddle[2][d1]) - (vektor.vMiddle[2][b1] - vektor.vMiddle[2][d1]) * (vektor.vMiddle[1][c1] - vektor.vMiddle[1][d1])) + (vektor.vMiddle[1][a1] - vektor.vMiddle[1][b1])
                          * ((vektor.vMiddle[2][b1] - vektor.vMiddle[2][d1]) * (vektor.vMiddle[0][c1] - vektor.vMiddle[0][d1]) - (vektor.vMiddle[0][b1] - vektor.vMiddle[0][d1]) * (vektor.vMiddle[2][c1] - vektor.vMiddle[2][d1])) + (vektor.vMiddle[2][a1] - vektor.vMiddle[2][b1])
                          * ((vektor.vMiddle[0][b1] - vektor.vMiddle[0][d1]) * (vektor.vMiddle[1][c1] - vektor.vMiddle[1][d1]) - (vektor.vMiddle[0][b1] - vektor.vMiddle[0][d1]) * (vektor.vMiddle[0][c1] - vektor.vMiddle[0][d1]))))) / 6;
          if (volume >= bigVolume) {
            *PbigVolume = volume;
          }
        }
      }
    }
  }
}
void callNotationInVektor() {   // the new vector gets asigned
  for (int i = 0; i < 100; i++)
  {
    asignValues ();
    asignValuesVector();
    y = lenholder;      // this random simulate the numbers ther acturly wil be used.
    lr.Data(y);              // the data gets ritton in to the libery linarRegression.
    x++;                     // for each y x counts 1 up. so you can numberise y.
    *Pwalue = y;             // whats Pwalue points as will be eaqul to y. that is to find the biggest y, ands its x, and the smallest
    ekstremCoefficients();   // finds bMax and bMin (biggest b form biggest y smallest b for smallest y.
  }
  bLinarRegression();        //calgulates the b in MAX(y=ax+b) and the b in MIN(y=ax+b.
  printLinarRegression();    //ther is one important line of code in that funcotion witch asigns walues
  lr.Reset();                // Reset the libary.
  small = 100, big = 0, walue = 0; x = 0;         //resets the shit
  vektor.newVektor[0][0] = (*Pa);
  vektor.newVektor[1][0] = (*Pb);       //b mittle // arra skal laves om
  vektor.newVektor[2][0] = (*Pr);
  vektor.newVektor[0][1] = (*Pa);
  vektor.newVektor[1][1] = (*Pbb); // b BIG
  vektor.newVektor[2][1] = (*Pr);
  vektor.newVektor[0][2] = (*Pa);
  vektor.newVektor[1][2] = (*Pbs); //b Small
  vektor.newVektor[2][2] = (*Pr);
}
void shortestDistanse() {       // finds the shortest distanse, form the New middle vector, to eny of the old middle vectors
  for (int ii = 0; ii < 10; ii++) { //there are 10 old middle vectors therfor the loop is runed throw 10 times
    dist = sqrt(pow(vektor.vMiddle[0][ii] - vektor.newVektor[0][0], 2) + pow(vektor.vMiddle[1][ii] - vektor.newVektor[1][0], 2) + pow(vektor.vMiddle[2][ii] - vektor.newVektor[2][0], 2));
    if (dist <= minDist) {          //finding the smalest dist and its number
      *PminDist = dist;
      *PminLenNr = counterLen;
    }
    counterLen++;                   //the counter is for noing witch one
  }
}
void volumen () {               //volume of the tetradron witch is all the 3 vectors form the new messurment, and the middle vector closets to
  vektor.newVektor[0][3] = vektor.vMiddle[0][minLenNr]; //the last array in new vector, is to be the vector closets to
  vektor.newVektor[1][3] = vektor.vMiddle[1][minLenNr];
  vektor.newVektor[2][3] = vektor.vMiddle[2][minLenNr];
  float xa11, ya11, za11;       float xb11, yb11, zb11;       float xc11, yc11, zc11;       float xd11, yd11, zd11; //the x y and z for all vectors ritton in to the arr
  float xa_b, ya_b, za_b; float xb_d, yb_d, zb_d; float xc_d, yc_d, zc_d; // (a-b) and so on....
  float xxPruduct, xyPruduct, xzPruduct;
  float dotPruduct;
  xa11 = vektor.newVektor[0][0]; xb11 = vektor.newVektor[0][1]; xc11 = vektor.newVektor[0][2]; xd11 = vektor.newVektor[0][3];
  ya11 = vektor.newVektor[1][0]; yb11 = vektor.newVektor[1][1]; yc11 = vektor.newVektor[1][2]; yd11 = vektor.newVektor[1][3];
  za11 = vektor.newVektor[2][0]; zb11 = vektor.newVektor[2][1]; zc11 = vektor.newVektor[2][2]; zd11 = vektor.newVektor[2][3];
  xa_b = xa11 - xb11; xb_d = xb11 - xd11; xc_d = xc11 - xd11;
  ya_b = ya11 - yb11; yb_d = yb11 - yd11; yc_d = yc11 - yd11;
  za_b = za11 - zb11; zb_d = zb11 - zd11; zc_d = zc11 - zd11;
  xxPruduct = yb_d * zc_d - zb_d * yc_d;
  xyPruduct = zb_d * xc_d - xb_d * zc_d;
  xzPruduct = xb_d * yc_d - yb_d * xc_d;
  dotPruduct = xa_b * xxPruduct + ya_b * xyPruduct + za_b * xzPruduct;
  if (dotPruduct < 0) {
    dotPruduct = dotPruduct * (-1);
  }
  volumeNew = (dotPruduct / 6);
}
void volumeCompare () {         // this function comperes the two volumes. If there are diffrens acktion will be taken
  float dif;
  dif = volumeNew / bigVolume;
  if (volumeNew <= bigVolume * (0.000000001)) { // the denigh factor
    vektor.vMiddle[0][minLenNr] = vektor.newVektor[0][0];    //the closest vector gets now replaced with the new vector
    vektor.vMiddle[1][minLenNr] = vektor.newVektor[1][0];
    vektor.vMiddle[2][minLenNr] = vektor.newVektor[2][0];
    vektor.vMiddle[0][minLenNr + 10] = vektor.newVektor[0][1]; //so does its bBig vector
    vektor.vMiddle[1][minLenNr + 10] = vektor.newVektor[1][1];
    vektor.vMiddle[2][minLenNr + 10] = vektor.newVektor[2][1];
    vektor.vMiddle[0][minLenNr + 20] = vektor.newVektor[0][2]; //ands its bSmall vector
    vektor.vMiddle[1][minLenNr + 20] = vektor.newVektor[1][2];
    vektor.vMiddle[2][minLenNr + 20] = vektor.newVektor[2][2];
    Serial.println("New data acceptet");
  }
  else {
    Serial.println("New data denighed");
  }
  matrixFunktions();
}

void matrixFunktions() {
  float aSum = 0, bmSum = 0;
  float Bb = 0, Bs = 1000;
  for (int ll = 0; ll < 10; ll++) {
    // avage a, bm
    aSum = aSum + vektor.vMiddle[0][ll];
    bmSum = bmSum + vektor.vMiddle[1][ll];
    if (vektor.vMiddle[1][ll + 10] >= Bb) { //finding the biggest of the big B
      Bb = vektor.vMiddle[1][ll + 10];
      *PbiggestBb = vektor.vMiddle[1][ll + 10];
    }
    if (vektor.vMiddle[1][ll + 20] <= Bs) { //finding the biggest of the big B
      Bs = vektor.vMiddle[1][ll + 20];
      *PsmallestBs = vektor.vMiddle[1][ll + 20];
    }
  }
  avagA = aSum / 10;
  avagBm = bmSum / 10;
}
void codeForTask1( void * parameter )
{
  for (;;) {
    //______________________________________________________
    switch (POGRAMMODE) {         // this swits purpus is to make sure that, the vMiddle array ar fild in stastart and then from than on only one vector set is cheket
      case 0:
        delay(1000);
        linarRegression();              // loads it starts walues to fill the array
        matrixFunktions();
        POGRAMMODE = 1;
        break;
      case 1:
        callNotationInVektor(); // loads its new 100 points and make it to 3 vectors
        shortestDistanse();     // form that New middle vector the shortest distanse to eny old middle vector is found
        volumen ();             // form the old middle vector, the new middle vector, big vector, samll vector. a tetradron i calgulatet
        callAria();             //calgulates the maximum tetradron volumen of 4 of all the vectors //int the end the old (main) tetradron is updatet and calgulation is repaetet
        volumeCompare ();       // than it compers the to tetradrons by a factor....
        matrixFunktions();
        break;
    }
    counter = 0;
    bigVolume = 0;
    counterLen = 0;
    minDist = 100000;
    delay(1);
    //______________________________________________________
  }
}
void codeForTask2( void * parameter )
{
  for (;;) {
    myTime = millis();
    if (button == 1) {
      digitalWrite(32, LOW);
    }
    else {
      digitalWrite(32, HIGH);
    }
    delay(1); //give CPU time to other prosses. AND STOPS THE WATCHS DORG INTERRUPT
    asignValues();
    asignValuesVector();
    if (lenholder > biggestBb * 3) { // the speed of the anglegrinder
      digitalWrite(32, HIGH);
      Serial.println("--------------!!!----!!!---!!!--KICKBACK DETECKTET--!!!---!!!----!!!--------------");
      delay(10000);
      button = 0;
    }
    if (lenholder < smallestBs / 3) { // the way the angle grinder is turned
      digitalWrite(32, HIGH);
      Serial.println("-----------------------ANGLEGRINDER TURNED RONG-----------------------");
      delay(10000);
      button = 0;
    }
  }
}
void loop() {
  delay(1);
}
