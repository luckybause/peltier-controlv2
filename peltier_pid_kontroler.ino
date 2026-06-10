// ============================================================
//  PID Peltier Controller v13 – Czysty PID + Self-Tune
//  Adafruit ItsyBitsy M0 (ATSAMD21G18)
// ============================================================
//  BIBLIOTEKI: Adafruit MAX31856, Adafruit BusIO, U8g2,
//              FlashStorage_SAMD
//
//  STEROWANIE: Czysty PID (bez FF)
//    Rampa:   jednostronne (tylko grzanie LUB tylko chłodzenie)
//    Cel:     pełny PID
//  SELF-TUNE: co 2s, 60 cykli = 2 minuty
//    Auto-start przy ON
//    Osobny Kp/Ki/Kd dla heat i cool
//    Zapisuje do profilu dla aktualnej temp i rampy
//  36 PROFILI: 9 temp x 4 rampy, interpolacja bilinearna
// ============================================================

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MAX31856.h>
#include <U8g2lib.h>
#include <FlashStorage_SAMD.h>

#define PIN_CS_TC  9
#define PIN_M1A    11
#define PIN_M1B    10
#define PIN_M2A    12   // wentylatory + (PWM predkosc)
#define PIN_M2B    7    // wentylatory -
#define PIN_POT1   A0
#define PIN_POT2   A1
#define PIN_BTN1   A4
#define PIN_BTN2   A5

#define PWM_MAX       255
#define TEMP_MIN_C    -15.0f
#define TEMP_MAX_DEF   110.0f
#define PID_DT_MS      100
#define INTEGRAL_MAX   400.0f
// Feed-forward rampy: moc PWM na kazdy 1 C/min zadanego rate.
// Wieksza wartosc = mocniejsze wyprzedzanie. 0 = wylaczone (czysty PID).
// Przy probelach z trzymaniem rate zwieksz; przy przeregulowaniu zmniejsz.
#define FF_GAIN        3.0f

#define SP_MIN    -15.0f
#define SP_MAX     100.0f
#define KP_MIN       1.0f
#define KP_MAX      30.0f
#define KI_MIN       0.0f
#define KI_MAX       3.0f
#define KD_MIN       0.0f
#define KD_MAX       3.0f
#define RAMP_MIN     0.5f
#define RAMP_MAX    40.0f
#define TMAX_MIN    50.0f
#define TMAX_MAX   115.0f

// Self-tune
#define ST_INT_MS   2000
#define ST_CYC_MAX    60
#define ST_HIST        6
#define ST_ADJ      0.04f
#define ST_DEAD     0.3f

// Cooldown
#define CD_TARGET   10.0f
// FREEZE: utrzymanie galu w stanie stalym do wymiany probki.
// Gal (galinstan) topi sie ~30C; trzymamy 20C z marginesem.
#define FREEZE_TARGET   20.0f   // docelowa temp galu (stan staly)
#define FREEZE_RAMP     6.0f    // lagodna rampa zejscia [C/min] (~2-3 min z 36->20)
#define FREEZE_TOL      0.8f    // tolerancja "stabilny" [C]
#define FREEZE_STABLE_MS 8000   // ile ms w tolerancji = "gal staly, gotowy"
#define CD_RAMP      5.0f
#define CD_TIMEOUT  120000UL

// Soft-start
#define SS_STEP     5
#define SS_INT     50
#define SS_INIT    20

// Profile
#define PT_N  9
#define PR_N  4
#define P_TOT (PT_N*PR_N)
const float PT[PT_N]={20,30,40,50,60,70,80,90,100};
const float PR[PR_N]={2,5,10,20};

// Konfigurowalna lista ramp do kalibracji (ustawiana z aplikacji).
// Domyslnie = PR. Maksymalnie 10 ramp.
#define CAL_RAMP_MAX 10
float calRamps[CAL_RAMP_MAX]={2,5,10,20};
int   calRampN=4;  // ile ramp aktywnych

struct Prof {
  float Kp_h,Ki_h,Kd_h;
  float Kp_c,Ki_c,Kd_c;
  bool  valid;
};
struct FD { bool cal; Prof p[P_TOT]; float ru,rd,tm; bool polSet; bool polSw; float calMin,calMax; };
FlashStorage(pidFlash,FD);

Adafruit_MAX31856 tc=Adafruit_MAX31856(PIN_CS_TC);
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0,U8X8_PIN_NONE);

Prof prof[P_TOT];
bool calDone=false;

float spT=25,spA=25;
float Kp_h=10,Ki_h=0.3f,Kd_h=0.8f;
float Kp_c=10,Ki_c=0.3f,Kd_c=0.3f;
float Kp=10,Ki=0.3f,Kd=0.8f;
float rU=2,rD=2,tMax=TEMP_MAX_DEF;
bool  htg=true;

float ig=0,pe=0,lT=25;
int   lPwm=0;
bool  tcE=false;

// ── Sterowanie z PC (zamiast potencjometrow/przyciskow) ──
bool  pcMode=true;       // true = wartosci z PC, ignoruj potencjometry
float calOffset=0.0f;    // offset kalibracji termopary [C]
String cmdBuf="";        // bufor komend Serial

// Self-tune
bool  stOn=false,stDone=false;
int   stC=0;
float stEH[ST_HIST]={},stPH[ST_HIST]={};
int   stI=0;
float stBH=999,stBKpH,stBKiH,stBKdH;
float stBC=999,stBKpC,stBKiC,stBKdC;
unsigned long stLt=0;
String stSt="";

// Soft-start
bool ssA=false; int ssPwm=0,ssTgt=0;
unsigned long ssTm=0;

// Cooldown
unsigned long cdT=0;

// Slope
#define SB 20
float slTb[SB]={};unsigned long slTm[SB]={};
int slI=0;bool slF=false;unsigned long slT=0;
String slSt="";

bool polSw=false;
bool polSet=false;  // czy polaryzacja zostala juz wykryta (zapisana we Flash)

enum St{MAN,AUTO,COOL,RTEST,CAL,FREEZE};
unsigned long frzStableT=0;  // od kiedy gal jest stabilny
bool frzReady=false;          // czy gal osiagnal staly stan

// Wentylatory (kanal M2: M2A=+, M2B=-)
int  fanSpeed=100;   // ustawiona predkosc w % (0-100)
bool fanOn=false;    // czy wentylatory wlaczone
St sys=MAN;

// Test rampy
int rtP=0;float rtU=0,rtD=0,rtT0=0;
unsigned long rtTm=0;String rtSt="";

// Kalibracja
int cTi=0,cRi=0,cPh=0,cIt=0;
unsigned long cPT=0;
float cTmn=50,cTmx=100;
#define CPM 10
float cTP[CPM];int cTN=0;
float cBH=999,cBC=999;
float cKpH,cKiH,cKdH,cKpC,cKiC,cKdC;
#define CH 10
float cEH[CH]={},cPwH[CH]={};int cHI=0;
unsigned long cLI=0;
String cSt="";
#define CA 300000  // dochodzenie do temp bazowej (5 min max)
#define CS 15000   // stabilizacja (15s)
#define CT 60000   // strojenie (60s = 30 iteracji)
#define CI  2000   // co 2s analizuj

unsigned long tP=0,tD=0,tR=0;
#define DT_D 200
#define DT_R 200

bool inM=false;int mP=0;
#define MI 8
const char* mL[]={"Cool Rate","Max Temp","Calibration","Ramp Test",
                  "Self-Tune","Save","Load","Reset"};

bool b1p=HIGH,b2p=HIGH;
uint32_t b1t=0,b2t=0;
bool b1h=false,b2h=false;
#define DB  50
#define HLD 800
#define HLL 2000

String fts(float v,int d){return String(v,d);}
float pot(int p){return 1.0f-(analogRead(p)/4095.0f);}
int pi_(int ti,int ri){return ti*PR_N+ri;}
int nTi(float t){int b=0;float bd=9999;for(int i=0;i<PT_N;i++){float d=abs(PT[i]-t);if(d<bd){bd=d;b=i;}}return b;}
int nRi(float r){int b=0;float bd=9999;for(int i=0;i<PR_N;i++){float d=abs(PR[i]-r);if(d<bd){bd=d;b=i;}}return b;}

void ldProf(float temp,float ramp){
  int ti0=0,ti1=0,ri0=0,ri1=0;
  for(int i=0;i<PT_N-1;i++) if(temp>=PT[i]&&temp<=PT[i+1]){ti0=i;ti1=i+1;break;}
  if(temp<PT[0]){ti0=ti1=0;}if(temp>PT[PT_N-1]){ti0=ti1=PT_N-1;}
  for(int i=0;i<PR_N-1;i++) if(ramp>=PR[i]&&ramp<=PR[i+1]){ri0=i;ri1=i+1;break;}
  if(ramp<PR[0]){ri0=ri1=0;}if(ramp>PR[PR_N-1]){ri0=ri1=PR_N-1;}
  float wt=(ti1!=ti0)?(temp-PT[ti0])/(PT[ti1]-PT[ti0]):0.5f;
  float wr=(ri1!=ri0)?(ramp-PR[ri0])/(PR[ri1]-PR[ri0]):0.5f;
  float kph=0,kih=0,kdh=0,kpc=0,kic=0,kdc=0;int cnt=0;
  auto add=[&](int ti,int ri,float w){
    Prof&p=prof[pi_(ti,ri)];
    if(p.valid){kph+=p.Kp_h*w;kih+=p.Ki_h*w;kdh+=p.Kd_h*w;
                kpc+=p.Kp_c*w;kic+=p.Ki_c*w;kdc+=p.Kd_c*w;cnt++;}
  };
  add(ti0,ri0,(1-wt)*(1-wr));add(ti0,ri1,(1-wt)*wr);
  add(ti1,ri0,wt*(1-wr));add(ti1,ri1,wt*wr);
  if(cnt>0){
    Kp_h=constrain(kph,KP_MIN,KP_MAX);Ki_h=constrain(kih,KI_MIN,KI_MAX);Kd_h=constrain(kdh,KD_MIN,KD_MAX);
    Kp_c=constrain(kpc,KP_MIN,KP_MAX);Ki_c=constrain(kic,KI_MIN,KI_MAX);Kd_c=constrain(kdc,KD_MIN,KD_MAX);
    if(htg){Kp=Kp_h;Ki=Ki_h;Kd=Kd_h;}else{Kp=Kp_c;Ki=Ki_c;Kd=Kd_c;}
    Serial.print("Prof: Kp=");Serial.print(Kp,1);Serial.print(" Ki=");Serial.print(Ki,2);Serial.print(" Kd=");Serial.println(Kd,2);
  }
}

void savF(){FD fd;fd.cal=calDone;for(int i=0;i<P_TOT;i++) fd.p[i]=prof[i];fd.ru=rU;fd.rd=rD;fd.tm=tMax;fd.polSet=polSet;fd.polSw=polSw;fd.calMin=cTmn;fd.calMax=cTmx;pidFlash.write(fd);Serial.println("Flash: zapisano.");}
void ldF(){FD fd;pidFlash.read(fd);if(fd.cal){calDone=true;for(int i=0;i<P_TOT;i++) prof[i]=fd.p[i];rU=fd.ru;rD=fd.rd;tMax=fd.tm;Serial.println("Flash: wczytano.");}if(fd.polSet){polSet=true;polSw=fd.polSw;}if(fd.calMin>=0&&fd.calMin<fd.calMax&&fd.calMax<=115){cTmn=fd.calMin;cTmx=fd.calMax;}}
void savePol(){FD fd;pidFlash.read(fd);fd.polSet=true;fd.polSw=polSw;pidFlash.write(fd);}
void rst(){calDone=false;for(int i=0;i<P_TOT;i++) prof[i]={10,0.3f,0.8f,10,0.3f,0.3f,false};Kp_h=Kp_c=Kp=10;Ki_h=Ki_c=Ki=0.3f;Kd_h=0.8f;Kd_c=Kd=0.3f;rU=rD=2;tMax=TEMP_MAX_DEF;ig=0;pe=0;Serial.println("Reset.");}

void wPwm(int o){lPwm=o;int h=o>0?o:0,c=o<0?-o:0;if(!polSw){analogWrite(PIN_M1A,h);analogWrite(PIN_M1B,c);}else{analogWrite(PIN_M1A,c);analogWrite(PIN_M1B,h);}}

// Wentylatory: M2A=+ dostaje PWM, M2B=- zawsze 0 (jeden kierunek obrotow)
void fanApply(){
  int pwm = fanOn ? (int)(fanSpeed*2.55f) : 0;  // % -> 0-255
  pwm = constrain(pwm,0,255);
  analogWrite(PIN_M2A, pwm);
  analogWrite(PIN_M2B, 0);
}
void stpPel(){analogWrite(PIN_M1A,0);analogWrite(PIN_M1B,0);lPwm=0;ssA=false;ssPwm=0;}
void setPwr(int o){
  o=constrain(o,-PWM_MAX,PWM_MAX);
  bool dir=(lPwm>0&&o<0)||(lPwm<0&&o>0),zero=(lPwm==0&&o!=0);
  if(dir||zero){if(dir){wPwm(0);delay(50);}ssA=true;ssTgt=o;ssPwm=(o>0)?SS_INIT:-SS_INIT;ssTm=millis();wPwm(ssPwm);}
  else if(ssA){ssTgt=o;}else{wPwm(o);}
}
void updSS(){
  if(!ssA) return;if(millis()-ssTm<SS_INT) return;ssTm=millis();
  if(ssTgt>0){ssPwm+=SS_STEP;if(ssPwm>=ssTgt){ssPwm=ssTgt;ssA=false;}}
  else if(ssTgt<0){ssPwm-=SS_STEP;if(ssPwm<=ssTgt){ssPwm=ssTgt;ssA=false;}}
  else{ssPwm=0;ssA=false;}wPwm(ssPwm);
}

#define TF 4
float tfB[TF]={25,25,25,25};int tfI=0;
float rdT(){
  uint8_t f=tc.readFault();if(f){tcE=true;return lT;}
  tcE=false;float raw=tc.readThermocoupleTemperature();
  // Ochrona przed NaN i bezsensownymi wartosciami
  if(isnan(raw)||raw<-50.0f||raw>200.0f){tcE=true;return lT;}
  float prev=tfB[(tfI-1+TF)%TF];if(abs(raw-prev)>8) raw=prev;
  tfB[tfI]=raw;tfI=(tfI+1)%TF;float s=0;for(int i=0;i<TF;i++) s+=tfB[i];
  lT=s/TF+calOffset;return lT;
}

void updRamp(){
  // Rampa: krok co DT_R ms. Przy DT_R=200ms jest 5 krokow/s = 300 krokow/min.
  // Krok = rate/300 daje dokladnie rate stopni na minute, ale plynnie (5x gestsze).
  float stepU=rU/300.0f, stepD=rD/300.0f;
  float d=spT-spA;
  if(abs(d)<0.02f){spA=spT;return;}
  if(d>0) spA=min(spA+stepU,spT);
  else    spA=max(spA-stepD,spT);
}

void updSlope(float temp){
  unsigned long now=millis();
  if(slT==0){slT=now;for(int i=0;i<SB;i++){slTb[i]=temp;slTm[i]=now;}return;}
  if(now-slT<1000) return;slT=now;
  slTb[slI]=temp;slTm[slI]=now;slI=(slI+1)%SB;if(slI==0) slF=true;
  int oi=slF?slI:0;float dt=(now-slTm[oi])/60000.0f;if(dt<0.05f) return;
  float act=(temp-slTb[oi])/dt,tgt=htg?rU:-rD;
  if(abs(spA-spT)<0.5f){slSt="";return;}
  float err=tgt-act;
  if(abs(err)<0.5f) slSt="OK";else if(err>0) slSt="+"+fts(err,1);else slSt=fts(err,1);
}

// ── Self-tune ─────────────────────────────────────────────────
void stStart(){
  stOn=true;stDone=false;stC=0;stLt=millis();stSt="Starting...";
  stBH=stBC=999;
  stBKpH=Kp_h;stBKiH=Ki_h;stBKdH=Kd_h;
  stBKpC=Kp_c;stBKiC=Ki_c;stBKdC=Kd_c;
  for(int i=0;i<ST_HIST;i++){stEH[i]=0;stPH[i]=0;}stI=0;
  ig=0;pe=0;
  Serial.print("ST START SP=");Serial.print(spT,1);Serial.print(" R=");Serial.println(rU,1);
}
void stStop(){
  stOn=false;stDone=true;
  Kp_h=stBKpH;Ki_h=stBKiH;Kd_h=stBKdH;
  Kp_c=stBKpC;Ki_c=stBKiC;Kd_c=stBKdC;
  if(htg){Kp=Kp_h;Ki=Ki_h;Kd=Kd_h;}else{Kp=Kp_c;Ki=Ki_c;Kd=Kd_c;}
  int idx=pi_(nTi(spT),nRi(htg?rU:rD));
  prof[idx]={Kp_h,Ki_h,Kd_h,Kp_c,Ki_c,Kd_c,true};
  calDone=true;ig=0;pe=0;
  stSt="OK zapisano";
  Serial.print("ST KONIEC Kp=");Serial.print(Kp,2);Serial.print(" Ki=");Serial.print(Ki,3);Serial.print(" Kd=");Serial.println(Kd,2);
  // Auto-zapis do Flash
  savF();
  Serial.println("Profil zapisany do Flash automatycznie");
}
void runST(float temp){
  if(!stOn||sys!=AUTO) return;
  unsigned long now=millis();if(now-stLt<ST_INT_MS) return;
  stLt=now;stC++;
  float err=spA-temp,ae=abs(err);
  stEH[stI]=err;stPH[stI]=(float)lPwm;stI=(stI+1)%ST_HIST;
  int sc=0;for(int i=0;i<ST_HIST-1;i++){int a=i,b=(i+1)%ST_HIST;if(stEH[a]*stEH[b]<0)sc++;}
  bool osc=(sc>=2);
  int sat=0;for(int i=0;i<ST_HIST;i++) if(abs(stPH[i])>=PWM_MAX-5) sat++;
  bool satd=(sat>=ST_HIST-1);
  int pi2=(stI-2+ST_HIST)%ST_HIST,ci2=(stI-1+ST_HIST)%ST_HIST;
  float tr=abs(stEH[ci2])-abs(stEH[pi2]);
  bool im=(tr<-0.1f),wo=(tr>0.3f);
  if(htg){
    if(osc){Kp_h=constrain(Kp_h*(1-ST_ADJ*1.5f),KP_MIN,KP_MAX);Kd_h=constrain(Kd_h*(1-ST_ADJ),KD_MIN,KD_MAX);Ki_h=constrain(Ki_h*(1-ST_ADJ*0.5f),KI_MIN,KI_MAX);ig*=0.5f;stSt="OSC-";}
    else if(satd&&ae>2){stSt="SAT";}
    else if(ae>8&&!im){Kp_h=constrain(Kp_h*(1+ST_ADJ*2),KP_MIN,KP_MAX);stSt="SLOW++";}
    else if(ae>3&&wo){Kp_h=constrain(Kp_h*(1+ST_ADJ),KP_MIN,KP_MAX);stSt="WORSE";}
    else if(ae>ST_DEAD&&im){Ki_h=constrain(Ki_h*(1+ST_ADJ*0.5f),KI_MIN,KI_MAX);stSt="Ki+";}
    else{stSt="OK";}
    Kp=Kp_h;Ki=Ki_h;Kd=Kd_h;
    if(ae<stBH){stBH=ae;stBKpH=Kp_h;stBKiH=Ki_h;stBKdH=Kd_h;}
  } else {
    if(osc){Kp_c=constrain(Kp_c*(1-ST_ADJ*1.5f),KP_MIN,KP_MAX);Kd_c=constrain(Kd_c*(1-ST_ADJ),KD_MIN,KD_MAX);Ki_c=constrain(Ki_c*(1-ST_ADJ*0.5f),KI_MIN,KI_MAX);ig*=0.5f;stSt="OSC-";}
    else if(satd&&ae>2){stSt="SAT";}
    else if(ae>8&&!im){Kp_c=constrain(Kp_c*(1+ST_ADJ*2),KP_MIN,KP_MAX);stSt="SLOW++";}
    else if(ae>3&&wo){Kp_c=constrain(Kp_c*(1+ST_ADJ),KP_MIN,KP_MAX);stSt="WORSE";}
    else if(ae>ST_DEAD&&im){Ki_c=constrain(Ki_c*(1+ST_ADJ*0.5f),KI_MIN,KI_MAX);stSt="Ki+";}
    else{stSt="OK";}
    Kp=Kp_c;Ki=Ki_c;Kd=Kd_c;
    if(ae<stBC){stBC=ae;stBKpC=Kp_c;stBKiC=Ki_c;stBKdC=Kd_c;}
  }
  Serial.print(now/1000.0f,1);Serial.print(",");Serial.print(temp,2);Serial.print(",");
  Serial.print(spA,2);Serial.print(",");Serial.print(spT,2);Serial.print(",");
  Serial.print(lPwm);Serial.print(",");Serial.print(Kp,3);Serial.print(",");
  Serial.print(Ki,4);Serial.print(",");Serial.print(Kd,3);Serial.println(",ST-"+stSt);
  if(stC>=ST_CYC_MAX) stStop();
}

// ── Czysty PID ────────────────────────────────────────────────
int compPID(float temp){
  float dt=PID_DT_MS/1000.0f,err=spA-temp;

  // Kierunek na podstawie rampy
  bool rH=(spT>(spA-1.0f));
  if(rH!=htg){
    ig=0;htg=rH;
    if(htg){Kp=Kp_h;Ki=Ki_h;Kd=Kd_h;}
    else   {Kp=Kp_c;Ki=Ki_c;Kd=Kd_c;}
  }

  // Dystans rampy do celu finalnego
  float spDistTgt=fabs(spT-spA);     // ile setpoint ma jeszcze do celu
  bool atT=(spDistTgt<0.5f);          // setpoint dobil do celu

  // ── WSPOLCZYNNIK PRZEJSCIA (0..1) ──────────────────────
  // Plynne wygaszanie feed-forward i mocy w poblizu celu.
  // Zamiast binarnego ciecia (skok!), moc maleje stopniowo
  // gdy setpoint zbliza sie do celu w ostatnich BLEND_C stopniach.
  const float BLEND_C=4.0f;
  float blend=1.0f;
  if(spDistTgt<BLEND_C) blend=spDistTgt/BLEND_C;  // liniowo 1->0

  // ── MIEKKI START RAMPY (0..1) ──────────────────────────
  // Feed-forward narasta przez pierwsze RAMP_RISE_MS od startu rampy,
  // zamiast wskakiwac od razu (likwiduje schodki na starcie).
  float soft=1.0f;
  unsigned long sinceRamp=millis()-tR;
  const unsigned long RAMP_RISE_MS=3000;
  if(sinceRamp<RAMP_RISE_MS) soft=(float)sinceRamp/RAMP_RISE_MS;

  // ── ANTI-WINDUP ────────────────────────────────────────
  // Integruj tylko gdy nie nasycamy wyjscia w te sama strone.
  // Zapobiega przeladowaniu integratora podczas dlugiej rampy,
  // ktory przy dojsciu do celu powodowal przestrzelenie i skok w dol.
  float igLim = INTEGRAL_MAX * (atT ? 1.0f : 0.5f); // mniejszy limit na rampie
  float igNew = ig + err*dt;
  igNew = constrain(igNew, -igLim, igLim);

  // Derywata
  float dv=(err-pe)/dt;pe=err;

  // ── FEED-FORWARD (plynny) ──────────────────────────────
  float ff=0;
  if(!atT){
    float rate=htg?rU:-rD;
    ff = rate*FF_GAIN*blend*soft;  // wygaszany przy celu i na starcie
  }

  // Wyjscie PID + FF
  float pidOut = Kp*err + Ki*igNew + Kd*dv;
  float out = pidOut + ff;

  // Anti-windup: cofnij integracje jesli wyjscie nasycone
  if((out>PWM_MAX && err>0) || (out<-PWM_MAX && err<0)){
    // nie powiekszaj integratora gdy juz na maksie
  } else {
    ig = igNew;  // akceptuj integracje tylko gdy nie nasycone
  }

  // ── MIEKKIE OGRANICZENIE MOCY PRZY CELU ────────────────
  // W strefie przejscia ogranicz max moc proporcjonalnie,
  // zeby nie bylo gwaltownego ruchu gdy temp dochodzi do celu.
  float maxOut = PWM_MAX;
  if(spDistTgt<BLEND_C && fabs(err)<2.0f){
    // blisko celu i maly blad - ogranicz moc plynnie
    maxOut = PWM_MAX*(0.4f + 0.6f*blend);
  }

  // Jednostronne sterowanie podczas rampy, dwustronne przy celu
  if(!atT){
    if(htg) out=constrain(out,0,maxOut);
    else    out=constrain(out,-maxOut,0);
  } else {
    out=constrain(out,-PWM_MAX,PWM_MAX);
  }

  return (int)out;
}

void detPol(){
  oled.clearBuffer();oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0,28,"Polarity check...");oled.drawStr(0,42,"Do not touch! 4s");oled.sendBuffer();
  delay(300);float t0=tc.readThermocoupleTemperature();
  polSw=false;analogWrite(PIN_M1A,80);analogWrite(PIN_M1B,0);delay(4000);
  float t1=tc.readThermocoupleTemperature();analogWrite(PIN_M1A,0);analogWrite(PIN_M1B,0);
  float d=t1-t0;if(d>=0.3f) polSw=false;else if(d<=-0.3f) polSw=true;
  polSet=true; savePol();  // zapisz polaryzacje na zawsze we Flash
  oled.clearBuffer();oled.setFont(u8g2_font_7x13B_tf);
  int nw=oled.getStrWidth(polSw?"SWAPPED":"NORMAL");oled.drawStr((128-nw)/2,28,polSw?"SWAPPED":"NORMAL");
  oled.setFont(u8g2_font_6x10_tf);char b[20];sprintf(b,"dT=%.2fC",d);
  int bw=oled.getStrWidth(b);oled.drawStr((128-bw)/2,46,b);oled.sendBuffer();delay(2000);
  Serial.println(polSw?"Pol:swapped":"Pol:normal");
}

void startRT(){sys=RTEST;rtP=0;rtU=rtD=0;rtT0=lT;rtTm=millis();rtSt="HEAT 0/60s";wPwm(PWM_MAX);}
void runRT(float t){
  if(sys!=RTEST) return;unsigned long el=millis()-rtTm;int s=(int)(el/1000);
  if(rtP==0){rtSt="HEAT "+String(s)+"/60s";if(t>=tMax-5||s>=60){wPwm(0);float dT=t-rtT0,dM=el/60000.0f;rtU=(dM>0)?dT/dM:0;rtP=1;rtT0=t;rtTm=millis();delay(300);wPwm(-PWM_MAX);}}
  else if(rtP==1){rtSt="COOL "+String(s)+"/60s";if(t<=TEMP_MIN_C+2||s>=60){wPwm(0);float dT=rtT0-t,dM=el/60000.0f;rtD=(dM>0)?dT/dM:0;if(rtU>0) rU=constrain(rtU*0.8f,RAMP_MIN,RAMP_MAX);if(rtD>0) rD=constrain(rtD*0.8f,RAMP_MIN,RAMP_MAX);rtP=2;sys=MAN;stpPel();rtSt="G:"+fts(rtU,1)+" C:"+fts(rtD,1);}}
}

void bldCP(){cTN=0;float t=cTmn;while(t<=cTmx+0.1f&&cTN<CPM){cTP[cTN++]=t;t+=10;}}
void stCalS(){sys=CAL;cPh=-1;cSt="Ustaw zakres";}
void stCalR(){
  bldCP();cTi=cRi=cPh=cIt=0;cPT=millis();cBH=cBC=999;
  cKpH=Kp_h;cKiH=Ki_h;cKdH=Kd_h;cKpC=Kp_c;cKiC=Ki_c;cKdC=Kd_c;
  for(int i=0;i<CH;i++){cEH[i]=cPwH[i]=0;}cHI=0;cLI=0;ig=0;pe=0;
  spT=cTP[0];spA=lT;rU=rD=calRamps[cRi];int tot=cTN*calRampN;char b[24];sprintf(b,"Start 1/%d",tot);cSt=String(b);
  Serial.println("=== KAL. START ===");
  // Wyslij plan kalibracji dla aplikacji: CALPLAN:tot,temps=...,ramps=...
  Serial.print("CALPLAN:");Serial.print(tot);
  Serial.print(",temps=");
  for(int i=0;i<cTN;i++){Serial.print(cTP[i],0);if(i<cTN-1)Serial.print("/");}
  Serial.print(",ramps=");
  for(int i=0;i<calRampN;i++){Serial.print(calRamps[i],0);if(i<calRampN-1)Serial.print("/");}
  Serial.println();
}
void savCP(){int ti=nTi(cTP[cTi]),idx=pi_(ti,nRi(calRamps[cRi]));prof[idx]={cKpH,cKiH,cKdH,cKpC,cKiC,cKdC,true};
  Serial.print("Prof T=");Serial.print(cTP[cTi],0);Serial.print(" R=");Serial.print(calRamps[cRi],0);Serial.print(" Kph=");Serial.println(cKpH,1);}
void nxtCS(){
  savCP();cRi++;if(cRi>=calRampN){cRi=0;cTi++;}
  int tot=cTN*calRampN,done=cTi*calRampN+cRi;
  if(cTi>=cTN){calDone=true;savF();sys=MAN;stpPel();char b[24];sprintf(b,"DONE %d/%d",tot,tot);cSt=String(b);Serial.println("=== KAL. ZAKONCZONA ===");return;}
  cPh=0;cPT=millis();cIt=0;cBH=cBC=999;cKpH=Kp_h;cKiH=Ki_h;cKdH=Kd_h;cKpC=Kp_c;cKiC=Ki_c;cKdC=Kd_c;
  for(int i=0;i<CH;i++){cEH[i]=cPwH[i]=0;}cHI=0;cLI=0;
  spT=cTP[cTi];rU=rD=calRamps[cRi];spA=lT;ig=0;pe=0;
  char b[24];sprintf(b,"Krok %d/%d",done+1,tot);cSt=String(b);
}
// ── KALIBRACJA ────────────────────────────────────────────
// Dla kazdego punktu (temp,rampa):
//   Faza 0: dochodzenie do temp bazowej z pelna moca (60s)
//   Faza 1: stabilizacja na temp bazowej (15s)
//   Faza 2: strojenie rampy w gore o 10C, potem w dol o 10C (60s)
//           Co 2s analizuje blad i koryguje Kp/Ki/Kd
//           Zapamietuje najlepsze parametry
//   Po fazie 2: przywraca najlepsze i przechodzi do kolejnego punktu
void runCal(float temp){
  if(sys!=CAL||cPh==-1) return;
  unsigned long now=millis(),el=now-cPT;
  float err=spA-temp,ae=abs(err);

  if(cPh==0){
    // Dochodzenie do temperatury bazowej
    if(now-tR>=DT_R){tR=now;updRamp();}
    setPwr(compPID(temp));
    char b[32];sprintf(b,"->%.0fC T=%.1f",cTP[cTi],temp);
    cSt=String(b);
    // Przejdz do stabilizacji gdy bliska celu lub timeout
    if(ae<2.0f||el>CA){
      cPh=1;cPT=now;cSt="Stabilizing...";
      ig=0;pe=0;
    }
  }
  else if(cPh==1){
    // Stabilizacja – trzymaj temperature bazowa
    setPwr(compPID(temp));
    cSt="Stabil "+String((CS-el)/1000)+"s";
    if(el>CS){
      // Przejdz do strojenia – ustaw rampe w gore o 10C
      cPh=2;cPT=now;cIt=0;cLI=0;
      cBH=cBC=999;
      cKpH=Kp_h;cKiH=Ki_h;cKdH=Kd_h;
      cKpC=Kp_c;cKiC=Ki_c;cKdC=Kd_c;
      for(int i=0;i<CH;i++){cEH[i]=cPwH[i]=0;}cHI=0;
      ig=0;pe=0;
      rU=rD=calRamps[cRi];
      // Cel: +10C w gore (lub -10C jesli za blisko tMax)
      if(cTP[cTi]+10<=tMax-5) spT=cTP[cTi]+10;
      else spT=cTP[cTi]-10;
      cSt="Tuning...";
    }
  }
  else if(cPh==2){
    // Strojenie – pracuje z aktywna rampa
    if(now-tR>=DT_R){tR=now;updRamp();}
    setPwr(compPID(temp));

    if(now-cLI>=CI){
      cLI=now;cIt++;
      // Zapisz do historii
      cEH[cHI]=err;cPwH[cHI]=(float)lPwm;cHI=(cHI+1)%CH;

      // Analiza oscylacji (3+ zmiany znaku w historii)
      int sc=0;
      for(int i=0;i<CH-1;i++){
        int a=i,b_=(i+1)%CH;
        if(cEH[a]*cEH[b_]<0) sc++;
      }
      bool osc=(sc>=3);

      // Saturacja PWM
      int sat=0;
      for(int i=0;i<CH;i++) if(abs(cPwH[i])>=PWM_MAX-5) sat++;
      bool satd=(sat>=CH-2);

      // Trend bledu
      int p2=(cHI-2+CH)%CH,c2=(cHI-1+CH)%CH;
      float tr=abs(cEH[c2])-abs(cEH[p2]);
      bool worse=(tr>0.3f);

      float adj=0.03f;

      // Strojenie aktywnego zestawu (heat lub cool)
      if(htg){
        if(osc){
          // Oscyluje – zmniejsz Kp,Kd
          Kp_h=constrain(Kp_h*(1-adj*1.5f),KP_MIN,KP_MAX);
          Kd_h=constrain(Kd_h*(1-adj),KD_MIN,KD_MAX);
          ig*=0.5f;
        } else if(satd&&ae>2){
          // Saturacja – nic nie da sie poprawic
        } else if(ae>8){
          Kp_h=constrain(Kp_h*(1+adj*2),KP_MIN,KP_MAX);
        } else if(ae>2&&worse){
          Kp_h=constrain(Kp_h*(1+adj),KP_MIN,KP_MAX);
          Kd_h=constrain(Kd_h*(1+adj*0.3f),KD_MIN,KD_MAX);
        } else if(ae>0.5f){
          Ki_h=constrain(Ki_h*(1+adj*0.5f),KI_MIN,KI_MAX);
        }
        Kp=Kp_h;Ki=Ki_h;Kd=Kd_h;
        if(ae<cBH){cBH=ae;cKpH=Kp_h;cKiH=Ki_h;cKdH=Kd_h;}
      } else {
        if(osc){
          Kp_c=constrain(Kp_c*(1-adj*1.5f),KP_MIN,KP_MAX);
          Kd_c=constrain(Kd_c*(1-adj),KD_MIN,KD_MAX);
          ig*=0.5f;
        } else if(satd&&ae>2){
        } else if(ae>8){
          Kp_c=constrain(Kp_c*(1+adj*2),KP_MIN,KP_MAX);
        } else if(ae>2&&worse){
          Kp_c=constrain(Kp_c*(1+adj),KP_MIN,KP_MAX);
          Kd_c=constrain(Kd_c*(1+adj*0.3f),KD_MIN,KD_MAX);
        } else if(ae>0.5f){
          Ki_c=constrain(Ki_c*(1+adj*0.5f),KI_MIN,KI_MAX);
        }
        Kp=Kp_c;Ki=Ki_c;Kd=Kd_c;
        if(ae<cBC){cBC=ae;cKpC=Kp_c;cKiC=Ki_c;cKdC=Kd_c;}
      }

      int tot=cTN*calRampN,done=cTi*calRampN+cRi+1;
      char b[32];
      sprintf(b,"%d/%d i%d e%.1f",done,tot,cIt,err);
      cSt=String(b);
      // Status kalibracji dla aplikacji PC
      Serial.print("CALSTAT:");Serial.print(done);Serial.print("/");
      Serial.print(tot);Serial.print(",T=");Serial.print(cTP[cTi],0);
      Serial.print(",R=");Serial.println(calRamps[cRi],0);

      // Log CSV
      Serial.print(now/1000.0f,1);Serial.print(",");
      Serial.print(temp,2);Serial.print(",");
      Serial.print(spA,2);Serial.print(",");
      Serial.print(spT,2);Serial.print(",");
      Serial.print(lPwm);Serial.print(",");
      Serial.print(Kp,3);Serial.print(",");
      Serial.print(Ki,4);Serial.print(",");
      Serial.print(Kd,3);
      Serial.print(",CAL-");Serial.println(done);
    }

    // Koniec fazy strojenia – zapisz najlepsze i nastepny krok
    if(el>CT){
      Kp_h=cKpH;Ki_h=cKiH;Kd_h=cKdH;
      Kp_c=cKpC;Ki_c=cKiC;Kd_c=cKdC;
      nxtCS();
    }
  }
}

void rdMP(){if(mP>1) return;float p=pot(PIN_POT1);if(mP==0) rD=RAMP_MIN+(RAMP_MAX-RAMP_MIN)*p;if(mP==1) tMax=TMAX_MIN+(TMAX_MAX-TMAX_MIN)*p;}

void hBtn(){
  uint32_t now=millis();bool r1=digitalRead(PIN_BTN1),r2=digitalRead(PIN_BTN2);
  if(r1==LOW&&b1p==HIGH&&(now-b1t)>DB){b1t=now;b1h=false;}
  if(r1==LOW&&!b1h&&(now-b1t)>=HLD){b1h=true;if(sys==RTEST){wPwm(0);sys=MAN;rtSt="Aborted";}else if(sys==CAL){stpPel();sys=MAN;cSt="Aborted";}else{inM=!inM;mP=0;}}
  if(r1==HIGH&&b1p==LOW){uint32_t h=now-b1t;if(!b1h&&h>DB&&h<HLD&&inM) mP=(mP+1)%MI;}
  b1p=r1;
  if(r2==LOW&&b2p==HIGH&&(now-b2t)>DB){b2t=now;b2h=false;}
  if(r2==LOW&&!b2h&&sys==COOL&&(now-b2t)>=HLL){b2h=true;stpPel();sys=MAN;ig=0;pe=0;Serial.println("Cooldown ANULOWANY");}
  if(r2==HIGH&&b2p==LOW){
    uint32_t h=now-b2t;if(b2h){b2h=false;b2p=r2;return;}if(h<DB){b2p=r2;return;}
    // BTN2 podczas wyboru zakresu kalibracji = start
    if(sys==CAL&&cPh==-1){b2p=r2;stCalR();return;}
    if(inM){
      switch(mP){
        case 2:inM=false;stCalS();break;case 3:inM=false;startRT();break;
        case 4:if(stOn)stStop();else if(sys==AUTO){stStart();inM=false;}break;
        case 5:savF();break;case 6:ldF();break;case 7:rst();break;
      }
    } else {
      if(sys==MAN){
        sys=AUTO;
        // Start zawsze od aktualnej temperatury (nie 30C) – brak skoku
        spA=lT;
        ig=0;pe=0;slT=0;tR=millis();
        if(calDone) ldProf(spT,rU);
        // BEZ auto-start self-tune – uzywaj zapisanych parametrow
        // Self-tune trzeba uruchomic recznie z menu
        Serial.println("ON");
      } else if(sys==AUTO){
        sys=COOL;cdT=millis();spT=CD_TARGET;spA=lT;ig=0;pe=0;slT=0;stOn=false;Serial.println("OFF – cooldown");
      }
    }
  }
  b2p=r2;
}

void drwMain(float temp){
  oled.clearBuffer();

  // ── PASEK GORNY (y=0-9): wskazniki po lewej, status po prawej ──
  oled.setFont(u8g2_font_5x7_tf);
  // Wskazniki stanu kalibracji - lewy gorny rog, maly font
  int xi=0;
  if(polSw)   { oled.drawStr(xi,7,"P"); xi+=8; }
  if(calDone) { oled.drawStr(xi,7,"C"); xi+=8; }
  if(stOn)    { oled.drawStr(xi,7,"ST"); xi+=14; }

  // Status ON/OFF/COOL - prawy gorny rog
  oled.setFont(u8g2_font_6x10_tf);
  if(sys==AUTO){ oled.drawBox(106,0,22,11); oled.setDrawColor(0); oled.drawStr(109,9,"ON"); oled.setDrawColor(1); }
  else if(sys==COOL){ oled.drawFrame(100,0,28,11); oled.drawStr(102,9,"CLD"); }
  else if(sys==FREEZE){ if(frzReady){oled.drawBox(94,0,34,11);oled.setDrawColor(0);oled.drawStr(96,9,"SOLID");oled.setDrawColor(1);}else{oled.drawFrame(96,0,32,11);oled.drawStr(98,9,"FRZ");} }
  else { oled.drawFrame(104,0,24,11); oled.drawStr(107,9,"OFF"); }

  // ── TEMPERATURA (y=12-32): duza czcionka, wlasna linia ──
  oled.setFont(u8g2_font_logisoso16_tf);
  oled.drawStr(0,32,(tcE?"ERR!":(fts(temp,1)+"C")).c_str());

  oled.drawHLine(0,36,128);

  // ── SETPOINT (y=46) ──
  oled.setFont(u8g2_font_6x10_tf);
  String sp="SET "+fts(spT,1)+"C";
  if(abs(spA-spT)>0.5f) sp+=" >"+fts(spA,0);   // spA shown only during ramp
  oled.drawStr(0,46,sp.c_str());

  // ── RAMP RATE (y=46, right side) ──
  String rl=fts(rU,0)+"/"+fts(rD,0);
  int rw=oled.getStrWidth(rl.c_str());
  oled.drawStr(128-rw,46,rl.c_str());

  // ── PWM BAR (y=54-62) ──
  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(0,61,lPwm>0?"HEAT":(lPwm<0?"COOL":"---"));
  int bw=abs(lPwm)*98/PWM_MAX;
  oled.drawFrame(30,54,98,9);
  if(bw>0) oled.drawBox(31,55,min(bw,96),7);

  oled.sendBuffer();
}
void drwST(float temp){
  oled.clearBuffer();oled.setFont(u8g2_font_6x10_tf);oled.drawStr(0,9,"SELF-TUNE");
  char cyc[20];sprintf(cyc,"%d/%d",stC,ST_CYC_MAX);
  int cw=oled.getStrWidth(cyc);oled.drawStr(128-cw,9,cyc);
  // Pasek postepu y=12-17
  int pg=stC*126/ST_CYC_MAX;oled.drawFrame(0,12,128,5);if(pg>0) oled.drawBox(1,13,min(pg,124),3);
  // Temperatura przesunieta nizej (y=20-38) zeby nie nachodzic na pasek
  oled.setFont(u8g2_font_10x20_tf);oled.drawStr(0,38,(fts(temp,1)+"C").c_str());
  oled.setFont(u8g2_font_6x10_tf);
  String es=String(htg?"H ":"C ")+"e:"+fts(spA-temp,1)+" Kp:"+fts(Kp,1);
  oled.drawStr(0,50,es.c_str());
  oled.drawStr(0,62,stSt.c_str());
  oled.sendBuffer();
}
void drwCalR(){
  oled.clearBuffer();float tMn=20+(90-20)*pot(PIN_POT1),tMx=30+(100-30)*pot(PIN_POT2);
  tMn=constrain((float)(round(tMn/10)*10),20,90);tMx=constrain((float)(round(tMx/10)*10),tMn+10,100);
  cTmn=tMn;cTmx=tMx;oled.setFont(u8g2_font_6x10_tf);
  const char* hdr="CALIBRATION RANGE";int hw=oled.getStrWidth(hdr);
  oled.drawStr((128-hw)/2,9,hdr);oled.drawHLine(0,12,128);
  oled.drawStr(0,28,"MIN (POT1)");oled.drawStr(0,44,"MAX (POT2)");
  oled.setFont(u8g2_font_10x20_tf);char bMn[8],bMx[8];sprintf(bMn,"%.0fC",tMn);sprintf(bMx,"%.0fC",tMx);
  int mnw=oled.getStrWidth(bMn),mxw=oled.getStrWidth(bMx);
  oled.drawStr(128-mnw,30,bMn);oled.drawStr(128-mxw,46,bMx);
  oled.setFont(u8g2_font_5x7_tf);oled.drawStr(0,62,"BTN2=start   BTN1=cancel");oled.sendBuffer();
}
void drwCal(float temp){
  if(cPh==-1){drwCalR();return;}
  oled.clearBuffer();oled.setFont(u8g2_font_6x10_tf);
  int tot=cTN*calRampN,done=cTi*calRampN+cRi;oled.drawStr(0,9,"CALIBRATION");
  int pg=(tot>0)?done*56/tot:0;oled.drawFrame(70,2,58,8);if(pg>0) oled.drawBox(71,3,min(pg,56),6);
  oled.drawHLine(0,12,128);
  // Temp duza (y=16-36)
  oled.setFont(u8g2_font_10x20_tf);oled.drawStr(0,36,(fts(temp,1)+"C").c_str());
  // Cel T/R - prawa strona, ponizej paska (y=28)
  oled.setFont(u8g2_font_6x10_tf);
  if(cTi<cTN){char b[24];sprintf(b,"T%.0f R%.0f",cTP[cTi],calRamps[cRi]);
    int bw=oled.getStrWidth(b);oled.drawStr(128-bw,28,b);}
  oled.drawStr(0,49,cSt.c_str());
  String ps=String(htg?"H":"C")+" Kp"+fts(Kp,1)+" Ki"+fts(Ki,2);oled.drawStr(0,61,ps.c_str());
  oled.sendBuffer();
}
void drwRT(float temp){
  oled.clearBuffer();oled.setFont(u8g2_font_6x10_tf);oled.drawStr(0,9,"RAMP TEST");oled.drawHLine(0,12,128);
  oled.setFont(u8g2_font_10x20_tf);oled.drawStr(0,36,(fts(temp,1)+"C").c_str());
  oled.setFont(u8g2_font_6x10_tf);oled.drawStr(0,49,rtSt.c_str());
  if(rtP==2) oled.drawStr(0,61,("G:"+fts(rtU,1)+" C:"+fts(rtD,1)+"C/m").c_str());
  else{unsigned long el=millis()-rtTm;int bw=(int)(min(el,60000UL)*126/60000UL);oled.drawFrame(0,55,128,8);if(bw>0) oled.drawBox(1,56,min(bw,124),6);}
  oled.sendBuffer();
}
void drwMenu(){
  oled.clearBuffer();oled.setFont(u8g2_font_6x10_tf);oled.drawStr(0,11,"<");oled.drawStr(122,11,">");
  oled.setFont(u8g2_font_7x13B_tf);int nw=oled.getStrWidth(mL[mP]);oled.drawStr((128-nw)/2,12,mL[mP]);oled.drawHLine(0,15,128);
  oled.setFont(u8g2_font_logisoso16_tf);String val;
  switch(mP){
    case 0:val=fts(rD,1)+"C/m";break;case 1:val=fts(tMax,0)+"C";break;
    case 2:val=calDone?"REDO":"START";break;case 3:val=rtP==2?(fts(rtU,1)+"/"+fts(rtD,1)):"START";break;
    case 4:val=stOn?"STOP":(sys==AUTO?"START":"ON+ST");break;
    case 5:val="SAVE";break;case 6:val="LOAD";break;case 7:val="RESET";break;
  }
  int vw=oled.getStrWidth(val.c_str());oled.drawStr((128-vw)/2,42,val.c_str());
  oled.setFont(u8g2_font_5x7_tf);
  const char* hint=mP<2?"POT1=adjust  BTN1=next":"BTN2=run   BTN1=next";
  int hw=oled.getStrWidth(hint);oled.drawStr((128-hw)/2,62,hint);oled.sendBuffer();
}

void setup(){
  Serial.begin(115200);analogReadResolution(12);
  pinMode(PIN_M1A,OUTPUT);pinMode(PIN_M1B,OUTPUT);analogWrite(PIN_M1A,0);analogWrite(PIN_M1B,0);
  pinMode(PIN_M2A,OUTPUT);pinMode(PIN_M2B,OUTPUT);analogWrite(PIN_M2A,0);analogWrite(PIN_M2B,0);
  pinMode(PIN_BTN1,INPUT_PULLUP);pinMode(PIN_BTN2,INPUT_PULLUP);
  if(!tc.begin()) Serial.println("ERROR: MAX31856!");
  tc.setThermocoupleType(MAX31856_TCTYPE_K);
  for(int i=0;i<P_TOT;i++) prof[i]={10,0.3f,0.8f,10,0.3f,0.3f,false};
  oled.begin();oled.clearBuffer();
  oled.setFont(u8g2_font_7x13B_tf);
  const char* t1="PID Peltier";int w1=oled.getStrWidth(t1);oled.drawStr((128-w1)/2,16,t1);
  oled.setFont(u8g2_font_6x10_tf);
  const char* t2="Pure PID + Self-Tune";int w2=oled.getStrWidth(t2);oled.drawStr((128-w2)/2,32,t2);
  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(8,48,"BTN2 = start");oled.drawStr(8,60,"hold BTN1 = menu");oled.sendBuffer();delay(1500);
  delay(200);float rt=tc.readThermocoupleTemperature();
  if(!isnan(rt)&&rt>-50&&rt<150){lT=rt;for(int i=0;i<TF;i++) tfB[i]=rt;}
  ldF();  // wczytaj Flash (w tym zapisana polaryzacje) PRZED detPol
  if(!polSet){
    // Polaryzacja jeszcze nie wykryta - wykryj raz i zapisz na zawsze
    detPol();
    rt=tc.readThermocoupleTemperature();if(!isnan(rt)&&rt>-50&&rt<150) lT=rt;
  } else {
    Serial.println(polSw?"Pol:swapped (z Flash)":"Pol:normal (z Flash)");
  }
  spA=spT=lT;
  Serial.println("czas_s,temp_C,setpoint_akt,setpoint_cel,PWM,Kp,Ki,Kd,stan");
  Serial.print("Start T=");Serial.println(lT,1);
  Serial.println("PC MODE - sterowanie z aplikacji");
  sendCfg();  // wyslij startowe nastawy do aplikacji
}

// ════════════════════════════════════════════════════════
//  PARSER KOMEND z PC (Serial)
//  Format: KOMENDA:wartosc\n  np. SP:25.5
// ════════════════════════════════════════════════════════
void sendCfg(){
  // Odeslij aktualne nastawy zeby aplikacja zsynchronizowala suwaki
  Serial.print("CFG:SP=");Serial.print(spT,2);
  Serial.print(",RU=");Serial.print(rU,2);
  Serial.print(",RD=");Serial.print(rD,2);
  Serial.print(",TMAX=");Serial.print(tMax,1);
  Serial.print(",KP=");Serial.print(Kp,3);
  Serial.print(",KI=");Serial.print(Ki,4);
  Serial.print(",KD=");Serial.print(Kd,3);
  Serial.print(",OFFSET=");Serial.print(calOffset,2);
  Serial.print(",STATE=");
  Serial.print(sys==AUTO?"AUTO":sys==COOL?"COOL":sys==CAL?"CAL":sys==RTEST?"RTEST":sys==FREEZE?"FREEZE":"MAN");
  Serial.print(",CAL=");Serial.print(calDone?1:0);
  Serial.print(",POL=");Serial.print(polSw?1:0);
  Serial.print(",POLSET=");Serial.print(polSet?1:0);
  Serial.print(",CALMIN=");Serial.print(cTmn,0);
  Serial.print(",CALMAX=");Serial.print(cTmx,0);
  Serial.print(",FAN=");Serial.print(fanOn?fanSpeed:0);
  Serial.println();
}

void procCmd(String c){
  c.trim();
  if(c.length()==0) return;
  int colon=c.indexOf(':');
  String key = (colon>=0)?c.substring(0,colon):c;
  String val = (colon>=0)?c.substring(colon+1):"";
  key.toUpperCase();
  float fv=val.toFloat();

  if(key=="SP"){ spT=constrain(fv,SP_MIN,SP_MAX); }
  else if(key=="RU"){ rU=constrain(fv,RAMP_MIN,RAMP_MAX); }
  else if(key=="RD"){ rD=constrain(fv,RAMP_MIN,RAMP_MAX); }
  else if(key=="TMAX"){ tMax=constrain(fv,TMAX_MIN,TMAX_MAX); }
  else if(key=="KP"){ Kp=constrain(fv,KP_MIN,KP_MAX); }
  else if(key=="KI"){ Ki=constrain(fv,KI_MIN,KI_MAX); }
  else if(key=="KD"){ Kd=constrain(fv,KD_MIN,KD_MAX); }
  else if(key=="OFFSET"){ calOffset=constrain(fv,-20.0f,20.0f); }
  else if(key=="START"){
    if(sys==MAN){
      sys=AUTO;spA=lT;ig=0;pe=0;slT=0;tR=millis();
      if(calDone) ldProf(spT,rU);
      Serial.println("ON");
    }
  }
  else if(key=="STOP"){
    if(sys==AUTO){ sys=COOL;cdT=millis();spT=CD_TARGET;spA=lT;ig=0;pe=0;slT=0;stOn=false;
      Serial.println("OFF - cooldown"); }
    else { stpPel();sys=MAN;Serial.println("STOP"); }
  }
  else if(key=="ESTOP"){ wPwm(0);stpPel();sys=MAN;stOn=false;Serial.println("E-STOP"); }
  else if(key=="FREEZE"){
    // Tryb zamrazania galu - lagodne zejscie do FREEZE_TARGET i AKTYWNE utrzymanie.
    // Nie wylacza Peltiera (zapobiega odbiciu ciepla i ponownemu stopieniu galu).
    sys=FREEZE; spT=FREEZE_TARGET; spA=lT; ig=0; pe=0; slT=0;
    rU=rD=FREEZE_RAMP; stOn=false; frzReady=false; frzStableT=0;
    Serial.print("FREEZE START -> target ");Serial.print(FREEZE_TARGET,0);
    Serial.println("C (gal solid)");
  }
  else if(key=="FREEZESTOP"){
    if(sys==FREEZE){ stpPel();sys=MAN;frzReady=false;Serial.println("FREEZE stopped"); }
  }
  else if(key=="FAN"){
    // FAN:0-100 - ustaw predkosc wentylatorow w %
    fanSpeed=constrain((int)fv,0,100);
    if(fanSpeed>0) fanOn=true;        // ustawienie >0 wlacza
    if(fanSpeed==0) fanOn=false;      // 0 wylacza
    fanApply();
    Serial.print("FAN ");Serial.print(fanOn?"ON ":"OFF ");Serial.print(fanSpeed);Serial.println("%");
  }
  else if(key=="FANON"){
    fanOn=true; if(fanSpeed==0) fanSpeed=100; fanApply();
    Serial.print("FAN ON ");Serial.print(fanSpeed);Serial.println("%");
  }
  else if(key=="FANOFF"){
    fanOn=false; fanApply();
    Serial.println("FAN OFF");
  }
  else if(key=="SELFTUNE"){ if(sys==AUTO) stStart(); }
  else if(key=="SELFTUNESTOP"){ if(stOn) stStop(); }
  else if(key=="AUTOCAL"){
    // Pelna automatyczna kalibracja wszystkich profili
    sys=CAL; cPh=-1;
    stCalR();  // start kalibracji od razu
    Serial.println("AUTOCAL START");
  }
  else if(key=="CALRANGE"){
    // CALRANGE:30,80 - ustaw zakres kalibracji (temp od, temp do)
    int cm=val.indexOf(',');
    if(cm>0){
      float lo=val.substring(0,cm).toFloat();
      float hi=val.substring(cm+1).toFloat();
      lo=constrain(lo,(float)TEMP_MIN_C,100.0f);
      hi=constrain(hi,lo+10.0f,115.0f);
      cTmn=lo; cTmx=hi;
      savF();  // zapisz zakres
      Serial.print("CALRANGE set: ");Serial.print(cTmn,0);
      Serial.print("-");Serial.println(cTmx,0);
    }
  }
  else if(key=="SETCALRAMPS"){
    // SETCALRAMPS:5,10,15,20 - ustaw liste ramp do kalibracji
    int n=0;
    String rest=val;
    while(n<CAL_RAMP_MAX && rest.length()>0){
      int cm=rest.indexOf(',');
      String tok=(cm>=0)?rest.substring(0,cm):rest;
      float r=tok.toFloat();
      if(r>=RAMP_MIN && r<=RAMP_MAX){ calRamps[n++]=r; }
      if(cm<0) break;
      rest=rest.substring(cm+1);
    }
    if(n>0){ calRampN=n; }
    Serial.print("CALRAMPS set: ");Serial.print(calRampN);Serial.print(" ramps: ");
    for(int i=0;i<calRampN;i++){Serial.print(calRamps[i],0);if(i<calRampN-1)Serial.print(",");}
    Serial.println();
  }
  else if(key=="REPOL"){
    // Wymus ponowne wykrycie polaryzacji
    polSet=false;
    detPol();
    Serial.println("Polaryzacja wykryta ponownie");
  }
  else if(key=="SETPOL"){
    // SETPOL:0 lub SETPOL:1 - reczne ustawienie polaryzacji
    polSw=(val.toInt()>0); polSet=true; savePol();
    Serial.println(polSw?"Pol:swapped (reczne)":"Pol:normal (reczne)");
  }
  else if(key=="AUTOCALSTOP"){
    if(sys==CAL){ stpPel(); sys=MAN; Serial.println("AUTOCAL ABORTED"); }
  }
  else if(key=="SAVE"){ savF(); }
  else if(key=="LOAD"){ ldF(); }
  else if(key=="RESET"){ rst(); }
  else if(key=="GET"){ sendCfg(); }
  else if(key=="DUMPCAL"){
    // Wyslij wszystkie profile do PC (do zapisu na dysku)
    // Format: PROF:idx,KpH,KiH,KdH,KpC,KiC,KdC,valid
    Serial.print("CALDUMP:");Serial.print(P_TOT);
    Serial.print(",cal=");Serial.println(calDone?1:0);
    for(int i=0;i<P_TOT;i++){
      Serial.print("PROF:");Serial.print(i);Serial.print(",");
      Serial.print(prof[i].Kp_h,3);Serial.print(",");
      Serial.print(prof[i].Ki_h,4);Serial.print(",");
      Serial.print(prof[i].Kd_h,3);Serial.print(",");
      Serial.print(prof[i].Kp_c,3);Serial.print(",");
      Serial.print(prof[i].Ki_c,4);Serial.print(",");
      Serial.print(prof[i].Kd_c,3);Serial.print(",");
      Serial.println(prof[i].valid?1:0);
    }
    Serial.println("CALDUMPEND");
  }
  else if(key=="SETPROF"){
    // SETPROF:idx,KpH,KiH,KdH,KpC,KiC,KdC,valid - ustaw jeden profil
    int idx=val.toInt();
    int c1=val.indexOf(',');
    if(idx>=0&&idx<P_TOT&&c1>0){
      String rest=val.substring(c1+1);
      float v[7]; int vi=0;
      while(vi<7){
        int cm=rest.indexOf(',');
        String tok=(cm>=0)?rest.substring(0,cm):rest;
        v[vi++]=tok.toFloat();
        if(cm<0) break;
        rest=rest.substring(cm+1);
      }
      if(vi>=6){
        prof[idx].Kp_h=v[0];prof[idx].Ki_h=v[1];prof[idx].Kd_h=v[2];
        prof[idx].Kp_c=v[3];prof[idx].Ki_c=v[4];prof[idx].Kd_c=v[5];
        prof[idx].valid=(vi>=7)?(v[6]>0.5f):true;
      }
    }
    return; // nie wysylaj sendCfg dla kazdego profilu (za duzo ruchu)
  }
  else if(key=="SETCALDONE"){
    // Po wgraniu wszystkich profili - oznacz kalibracje jako gotowa i zapisz
    calDone=(val.toInt()>0);
    savF();
    Serial.println("Kalibracja wgrana z PC");
  }
  else if(key=="PROFILE"){
    // PROFILE:temp,rampa,Kp,Ki,Kd - ustaw profil dla danej temp/rampy
    // (uproszczone - ustawia biezace Kp/Ki/Kd)
    int p1=val.indexOf(','),p2=val.indexOf(',',p1+1);
    int p3=val.indexOf(',',p2+1),p4=val.indexOf(',',p3+1);
    if(p1>0&&p2>0&&p3>0&&p4>0){
      spT=constrain(val.substring(0,p1).toFloat(),SP_MIN,SP_MAX);
      rU =constrain(val.substring(p1+1,p2).toFloat(),RAMP_MIN,RAMP_MAX);
      Kp =constrain(val.substring(p2+1,p3).toFloat(),KP_MIN,KP_MAX);
      Ki =constrain(val.substring(p3+1,p4).toFloat(),KI_MIN,KI_MAX);
      Kd =constrain(val.substring(p4+1).toFloat(),KD_MIN,KD_MAX);
    }
  }
  // Po kazdej komendzie odeslij potwierdzenie stanu
  sendCfg();
}

void readSerial(){
  while(Serial.available()){
    char ch=Serial.read();
    if(ch=='\n'||ch=='\r'){
      if(cmdBuf.length()>0){ procCmd(cmdBuf); cmdBuf=""; }
    } else {
      cmdBuf+=ch;
      if(cmdBuf.length()>64) cmdBuf=""; // ochrona przed przepelnieniem
    }
  }
}

void loop(){
  uint32_t now=millis();
  readSerial();              // czytaj komendy z PC
  if(!pcMode) hBtn();        // przyciski tylko gdy NIE w trybie PC
  if(!pcMode && inM) rdMP();
  else if(!pcMode && (sys==AUTO||sys==MAN)){spT=SP_MIN+(SP_MAX-SP_MIN)*pot(PIN_POT1);rU=RAMP_MIN+(RAMP_MAX-RAMP_MIN)*pot(PIN_POT2);}
  if(sys==AUTO&&!inM&&(now-tR>=DT_R)){tR=now;updRamp();}
  if(sys==AUTO&&!inM) updSlope(lT);else slT=0;
  if(sys==AUTO) runST(lT);
  if(sys==CAL) runCal(lT);
  if(sys==RTEST) runRT(lT);
  updSS();
  if(now-tP>=PID_DT_MS){
    tP=now;float temp=rdT();
    if(temp>tMax&&sys!=MAN){stpPel();sys=COOL;cdT=now;spT=CD_TARGET;spA=temp;ig=0;pe=0;Serial.println("!!! TEMP MAX !!!");}
    switch(sys){
      case AUTO:
        if(temp<TEMP_MIN_C&&lPwm<0) stpPel();else setPwr(compPID(temp));
        Serial.print(now/1000.0f,1);Serial.print(",");Serial.print(temp,2);Serial.print(",");
        Serial.print(spA,2);Serial.print(",");Serial.print(spT,2);Serial.print(",");
        Serial.print(lPwm);Serial.print(",");Serial.print(Kp,3);Serial.print(",");
        Serial.print(Ki,4);Serial.print(",");Serial.print(Kd,3);Serial.println(",AUTO");
        break;
      case COOL:
        if(now-cdT>=CD_TIMEOUT){stpPel();sys=MAN;break;}
        if(spA>CD_TARGET) spA=max(spA-rD/60.0f,CD_TARGET);
        if(lT<=CD_TARGET+1){stpPel();sys=MAN;Serial.println("Cooldown done");break;}
        setPwr(compPID(temp));
        Serial.print(now/1000.0f,1);Serial.print(",");Serial.print(temp,2);Serial.print(",");
        Serial.print(spA,2);Serial.print(",");Serial.print(CD_TARGET,1);Serial.print(",");
        Serial.print(lPwm);Serial.print(",");Serial.print(Kp,3);Serial.print(",");
        Serial.print(Ki,4);Serial.print(",");Serial.print(Kd,3);Serial.println(",COOLDOWN");
        break;
      case FREEZE: {
        // Lagodna rampa setpointu w dol do FREEZE_TARGET
        updRamp();
        // AKTYWNE utrzymanie - PID caly czas pracuje, NIE wylaczamy Peltiera.
        // To kluczowe: po wylaczeniu cieplo z radiatora odbilo by gal do plynnego.
        setPwr(compPID(temp));
        // Wykryj czy gal jest stabilnie zimny (gotowy do wymiany probki)
        if(fabs(temp-FREEZE_TARGET)<=FREEZE_TOL){
          if(frzStableT==0) frzStableT=now;
          else if(!frzReady && (now-frzStableT)>=FREEZE_STABLE_MS){
            frzReady=true;
            Serial.println("FREEZE READY - gal solid, mozna wymienic probke");
          }
        } else {
          frzStableT=0;  // wyszlo z tolerancji - reset licznika
          if(frzReady){ frzReady=false; }
        }
        Serial.print(now/1000.0f,1);Serial.print(",");Serial.print(temp,2);Serial.print(",");
        Serial.print(spA,2);Serial.print(",");Serial.print(FREEZE_TARGET,1);Serial.print(",");
        Serial.print(lPwm);Serial.print(",");Serial.print(Kp,3);Serial.print(",");
        Serial.print(Ki,4);Serial.print(",");Serial.print(Kd,3);
        Serial.println(frzReady?",FREEZE_READY":",FREEZE");
        break;
      }
      case MAN:
        stpPel();
        // Wysylaj dane nawet w trybie MAN - logger widzi temperature na zywo
        Serial.print(now/1000.0f,1);Serial.print(",");
        Serial.print(temp,2);Serial.print(",");
        Serial.print(lT,2);Serial.print(",");
        Serial.print(spT,2);Serial.print(",");
        Serial.print(0);Serial.print(",");
        Serial.print(Kp,3);Serial.print(",");
        Serial.print(Ki,4);Serial.print(",");
        Serial.print(Kd,3);Serial.println(",MAN");
        break;
      default:break;
    }
  }
  if(now-tD>=DT_D){
    tD=now;
    if(sys==CAL) drwCal(lT);else if(sys==RTEST) drwRT(lT);
    else if(stOn&&sys==AUTO) drwST(lT);else if(inM) drwMenu();else drwMain(lT);
  }
}
