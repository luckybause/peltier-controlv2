// ============================================================
//  PID Peltier Controller v13 – Czysty PID + Self-Tune
//  Adafruit ItsyBitsy M0 (ATSAMD21G18)
// ============================================================
//  BIBLIOTEKI: Adafruit MAX31856, Adafruit BusIO, U8g2,
//              FlashStorage_SAMD
//
//  STEROWANIE: PID + feed-forward na rampie
//    Rampa:   jednostronne (tylko grzanie LUB tylko chłodzenie), FF niesie
//             wiekszosc ciezaru, PID tylko dokorygowuje
//    Cel:     pełny PID (FF wylaczony po osiagnieciu celu)
//    Anti-windup: calka w jednostkach WYJSCIA, klamrowana do prawdziwego
//             zakresu aktuatora (0..PWM_MAX) zamiast do stalej - technika
//             z Arduino-PID-Library (Beauregard) / QuickPID
//    Pochodna: liczona z POMIARU (temperatury), nie z bledu - eliminuje
//             "derivative kick" przy skokowych zmianach celu/profilu
//  SELF-TUNE: co 2s, 60 cykli = 2 minuty
//    Start RECZNY z menu/apki (SELFTUNE) - NIE auto-startuje przy ON, zeby
//    przy kazdym wlaczeniu nie nadpisywac juz zapisanych/skalibrowanych nastaw
//    Osobny Kp/Ki/Kd dla heat i cool
//    Zapisuje do profilu dla aktualnej temp i rampy
//  81 PROFILE: 9 temp x 9 ramp (5/10/20/30/40/50/60/70/80 C/min),
//    interpolacja bilinearna. Kazdy punkt: RELAY (baza) + RAMP TEST
//    (auto-dostrajanie self-tune podczas grzania z zadana rampa)
// ============================================================

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MAX31856.h>
#include <U8g2lib.h>
#include <FlashStorage_SAMD.h>

#define PIN_CS_TC  9
#define PIN_CS_TC2 A1   // CS drugiej termopary (pomiar dodatkowy, nie wplywa na PID)
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
// (dawniej byla tu INTEGRAL_MAX jako sztywny limit calki - usunieta, patrz
// komentarz przy calkowaniu w compPID(): teraz limit calki to PRAWDZIWY
// zakres wyjscia (0..PWM_MAX), nie arbitralna stala).
//
// METODYKA KIERUNKU (na zyczenie): grzanie = normalny PID w gore/w dol PWM.
// "Chlodzenie" = PWM samo spada w kierunku zera (chlodzenie PASYWNE,
// oddawanie ciepla do otoczenia) - to sie dzieje samo, bez osobnego
// mechanizmu. Aktywne chlodzenie (ujemny PWM, odwrocenie Peltiera) wlacza
// sie NATYCHMIAST, bez zadnego opoznienia/marginesu, w momencie gdy SUROWE
// (nieprzyciete) wyjscie PID faktycznie zejdzie ponizej zera - czyli
// dokladnie tam, gdzie musimy odwrocic polaryzacje. Patrz blok kierunku w
// compPID() (po policzeniu "out", przed twardym obcieciem 0..PWM_MAX).
// Feed-forward rampy: moc PWM na kazdy 1 C/min zadanego rate.
// Wieksza wartosc = mocniejsze wyprzedzanie. 0 = wylaczone (czysty PID).
// Przy probelach z trzymaniem rate zwieksz; przy przeregulowaniu zmniejsz.
#define FF_GAIN        3.0f

#define SP_MIN    -15.0f
#define SP_MAX     100.0f
#define KP_MIN       1.0f
#define KP_MAX      30.0f
#define KI_MIN       0.0f
#define KI_MAX       1.5f
#define KD_MIN       0.0f
#define KD_MAX      80.0f
// Bazowe wartosci PID - od nich startuje strojenie KAZDEGO punktu kalibracji.
// KLUCZOWE: bez resetu do bazy kolejne punkty dziedziczyly napompowane Ki
// z poprzednich -> Ki tylko roslo, ladowalo na maksimum wszedzie.
#define KP_BASE      10.0f
#define KI_BASE      0.3f
#define KD_BASE_H    0.8f
#define KD_BASE_C    0.3f
#define RAMP_MIN     0.5f
#define RAMP_MAX    80.0f
#define TMAX_MIN    50.0f
#define TMAX_MAX   115.0f

// Self-tune
#define ST_INT_MS   2000
#define ST_CYC_MAX    60
#define ST_HIST        6
#define ST_ADJ      0.04f
#define ST_DEAD     0.3f

// STOP: wylacz Peltier od razu, wentylatory chodza jeszcze chwile
// (dochladzaja radiator, potem same gasna - jesli byly wlaczone)
#define FAN_RUNON_MS 120000UL  // czas pracy wentylatorow po STOP [ms] = 2 min
// FREEZE: utrzymanie galu w stanie stalym do wymiany probki.
// Gal (galinstan) topi sie ~30C; trzymamy 20C z marginesem.
#define FREEZE_TARGET   20.0f   // docelowa temp galu (stan staly)
#define FREEZE_RAMP     6.0f    // lagodna rampa zejscia [C/min] (~2-3 min z 36->20)
#define FREEZE_TOL      0.8f    // tolerancja "stabilny" [C]
#define FREEZE_STABLE_MS 8000   // ile ms w tolerancji = "gal staly, gotowy"

// Soft-start
#define SS_STEP     5
#define SS_INT     50
#define SS_INIT    20

// Profile
#define PT_N  9
#define PR_N  9
#define P_TOT (PT_N*PR_N)
const float PT[PT_N]={20,30,40,50,60,70,80,90,100};
const float PR[PR_N]={5,10,20,30,40,50,60,70,80};

// Konfigurowalna lista ramp do kalibracji (ustawiana z aplikacji).
// Domyslnie = PR. Maksymalnie 10 ramp.
#define CAL_RAMP_MAX 20
float calRamps[CAL_RAMP_MAX]={5,10,20,30,40,50,60,70,80};
int   calRampN=9;  // ile ramp aktywnych

struct Prof {
  float Kp_h,Ki_h,Kd_h;
  float Kp_c,Ki_c,Kd_c;
  bool  valid;
};
// FLASH_VER: bump gdy zmienia sie bajtowy layout FD/Prof (np. zmiana P_TOT -
// tak jak tutaj: PR_N 8->9). Stare dane w Flash NIE pasuja bajt-w-bajt do
// nowego struct (przesuwaja sie offsety WSZYSTKICH pol PO tablicy p[]) - bez
// tej kontroli ldF() czytalby smieci (losowe Kp/Ki/Kd, polSw, zakres
// kalibracji) zamiast poprawnie je zignorowac. Patrz ldF().
#define FLASH_VER 2u
struct FD { uint32_t ver; bool cal; Prof p[P_TOT]; float ru,rd,tm; bool polSet; bool polSw; float calMin,calMax; };
FlashStorage(pidFlash,FD);

Adafruit_MAX31856 tc=Adafruit_MAX31856(PIN_CS_TC);
Adafruit_MAX31856 tc2=Adafruit_MAX31856(PIN_CS_TC2);  // druga termopara (pomiar dodatkowy)
float temp2=NAN;      // odczyt z drugiej termopary
bool tc2OK=false;     // czy druga termopara dziala
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0,U8X8_PIN_NONE);

Prof prof[P_TOT];
bool calDone=false;

float spT=25,spA=25;
float Kp_h=10,Ki_h=0.3f,Kd_h=0.8f;
float Kp_c=10,Ki_c=0.3f,Kd_c=0.3f;
float Kp=10,Ki=0.3f,Kd=0.8f;
float rU=2,rD=2,tMax=TEMP_MAX_DEF;
bool  htg=true;
float dFilt=0;        // filtrowana pochodna (tlumi szum termopary)
float pwmFilt=0;      // filtrowane wyjscie PWM (wygladza skoki mocy)

float ig=0,peT=25,lT=25;
// dInit: czy peT (ostatnia temperatura do pochodnej) jest juz "swieza" po
// resecie (ig=0;dInit=false w kodzie). Bez tego pierwszy cykl PID po kazdym
// resecie liczylby pochodna wzgledem STAREJ/zerowej wartosci peT - falszywy,
// czasem ogromny skok pochodnej na jedna probke. Patrz komentarz w compPID().
bool dInit=false;
int   lPwm=0;
bool  tcE=false;

// ── KODY BLEDOW (ERR:code=N,...,active=0/1) - wysylane na zbocze (raz przy
//    wystapieniu, raz przy ustapieniu), zeby nie zapchac Serial. Apka PC
//    dekoduje kod na czytelny opis i pokazuje w panelu diagnostyki.
//    1=fault termopary glownej (bitmaska SR MAX31856)
//    2=odczyt poza zakresem/NaN (fault=0, mozliwy szum/zakloc)
//    3=TEMP MAX - bezpieczne zatrzymanie
//    4=MAX31856 nie odpowiada przy starcie (SPI/polaczenie)
bool tcErrSent=false, tcRangeErrSent=false;

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

// STOP fan run-on (wentylatory dochladzaja po STOP)
unsigned long fanRunonT=0;
bool fanRunonActive=false;

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
float cTmn=20,cTmx=90;
#define CPM 10
float cTP[CPM];int cTN=0;
float cBH=999,cBC=999;
float cKpH,cKiH,cKdH,cKpC,cKiC,cKdC;
// cLastOk: czy OSTATNI test relay dla biezacej temperatury naprawde sie
// udal (prawdziwy pomiar Ku/Tu), czy tez sciagnal do wartosci bazowych
// (KP_BASE itd. - patrz "RELAY FAIL - bazowe" w runCal). Uzywane w savCP()
// do poprawnego ustawienia Prof.valid - patrz komentarz przy savCP().
bool cLastOk=false;
#define CH 10
float cEH[CH]={},cPwH[CH]={};int cHI=0;
unsigned long cLI=0;
String cSt="";
#define CA 300000  // dochodzenie do temp bazowej (5 min max) - uzywane tez do cofniecia sie przed testem rampy
#define CS 15000   // stabilizacja (15s)
#define CT 60000   // strojenie (60s = 30 iteracji) - dlugosc jednego testu rampowania
#define CI  2000   // co 2s analizuj/dostrajaj w trakcie testu rampowania

// ── TEST ROMPOWANIA per RAMPA (po relay, na zyczenie) ──────────────
// Relay (wyzej) mierzy TYLKO bazowy Kp/Ki/Kd dla danej temperatury - rampa
// nie ma na to wplywu (savCP() kopiuje ten sam wynik do wszystkich ramp).
// Ten test to NAPRAWIA: dla KAZDEJ rampy z calRamps[] osobno grzejemy z tym
// konkretnym rampem do celu i patrzymy, czy AT trzyma sie ASP (spA) - jesli
// nie, dostrajamy Kp/Ki/Kd (identyczna heurystyka co self-tune, patrz
// runST() - oscylacje/nasycenie/za wolno/gorzej - ale WLASNA kopia bo dziala
// w CAL, nie w AUTO). Tylko galaz GRZANIA (Kp_h/Ki_h/Kd_h) - to jest
// jednoznacznie test grzania, cool zostaje z relay. Kazda rampa startuje OD
// NOWA od bazowego profilu z relay (nie dziedziczy po poprzedniej rampie) -
// powtarzalnosc, kazdy wynik niezalezny od kolejnosci testowania.
#define RAMP_TEST_DROP    8.0f   // o ile C cofnac sie przed testem, zeby bylo co wjezdzac (updRamp() nic nie robi gdy spA juz ==spT)
#define RAMP_TEST_OK_ERR  1.5f   // najlepszy |AT-ASP| w trakcie testu musi spasc ponizej tego, zeby wynik uznac za udany
unsigned long cRT=0;             // ostatnia analiza/dostrojenie w tescie rampowania (co CI ms)
float cRampBestErr=999;
float cRampBestKpH=0,cRampBestKiH=0,cRampBestKdH=0;

// ── RELAY FEEDBACK AUTOTUNING (Astrom-Hagglund + Tyreus-Luyben) ──
// Standard przemyslowy. Zamiast zgadywac nastawy, MIERZY charakterystyke
// ukladu (ultimate gain Ku, ultimate period Tu) wymuszajac kontrolowana
// oscylacje przekaznikiem, potem LICZY Kp/Ki/Kd ze wzorow.
#define RELAY_AMP    60       // startowa amplituda przekaznika [PWM] (~24% mocy - lagodne pobudzenie)
// Jesli PRZY STALEJ amplitudzie temperatura nigdy nie przekracza setpointu w
// obie strony (blisko otoczenia na dole zakresu, blisko granicy mocy grzania
// na gorze zakresu), test wisi po jednej stronie i NIGDY nie zaliczy cyklu -
// zaden timeout tego nie naprawi. Dlatego amplituda ESKALUJE: jesli przez
// RELAY_ESCALATE_MS nie zlapiemy ani jednego prawdziwego przejscia, mocniej
// pobudzamy uklad. relayAmpCur to biezaca (mozliwe ze juz podbita) amplituda
// dla aktualnego punktu; resetowana do RELAY_AMP na starcie kazdego punktu.
#define RELAY_AMP_MAX   140    // sufit eskalacji [PWM] (~55% mocy)
#define RELAY_AMP_STEP   40    // o ile podbic przy kazdej eskalacji
#define RELAY_ESCALATE_MS 90000  // po ilu ms bez cyklu podbic amplitude (90s)
float relayAmpCur=RELAY_AMP;
#define RELAY_HYST   0.3f     // histereza [C] - martwa strefa wokol setpointu
#define RELAY_CYCLES 6        // ile ostatnich cykli trzymamy w buforze do usredniania
// Ile PRAWDZIWYCH przejsc przez setpoint potrzeba zeby zakonczyc test
// (RELAY_CYCLES do bufora + margines, zeby odrzucic niestabilne pierwsze cykle)
#define RELAY_EXIT_CYCLES (RELAY_CYCLES+2)
// UWAGA: byla to wczesniej przyczyna masowych "RELAY FAIL - bazowe" - uklad
// termiczny (blok + Peltier) potrafi miec okres oscylacji rzedu dziesiatek
// sekund do kilku minut przy tak lagodnej amplitudzie (24%). 3 minuty na caly
// test prawie nigdy nie wystarczaly zeby zlapac 8 prawdziwych przejsc, wiec
// test prawie zawsze wpadal w timeout i kalibracja cicho spadala do wartosci
// bazowych. Zwiekszone do 10 min - jesli u Ciebie i tak zawsze timeout'uje,
// zwieksz dalej albo zmniejsz RELAY_EXIT_CYCLES.
#define RELAY_MAX_MS 600000    // max czas relay testu (10 min) - zabezpieczenie
// Przejscia przez setpoint szybsze niz to [ms] sa odrzucane jako szum
// (drgania odczytu termopary, zaklocenia PWM sterownika Peltiera) - NIE licza
// sie jako prawdziwy cykl oscylacji. Aluminiowy blok + TEC ma bezwladnosc
// ciepla - realny okres oscylacji rzedu pojedynczych sekund jest fizycznie
// bardzo malo prawdopodobny. To jest dokladnie to, co sie stalo w Twojej
// ostatniej kalibracji dla 40C: firmware "zlapal" Tu≈7.3s (wyliczone wstecz
// z Ki=0.79 i Kd=14.69), co przy PID DT=100ms i histerezie 0.3C wyglada na
// szum, nie prawdziwy cykl termiczny - stad ten jeden wynik w tabeli mogl
// byc rownie bezwartosciowy jak jawne "bazowe" fallbacki. Jesli po tej
// zmianie kalibracja u Ciebie w ogole nie lapie cykli, zmniejszaj to
// stopniowo; jesli nadal widzisz podejrzanie krotkie Tu w wynikach, zwieksz.
#define RELAY_MIN_PERIOD_MS 8000
float relayPeakHi=-999,relayPeakLo=999;  // szczyty oscylacji w biezacym cyklu
float relayAmps[RELAY_CYCLES]={};        // amplitudy kolejnych cykli
float relayPers[RELAY_CYCLES]={};        // okresy kolejnych cykli
unsigned long relayTcross=0;  // czas ostatniego przejscia przez setpoint
int relayCycN=0;              // licznik zmierzonych cykli
bool relayState=false;        // stan przekaznika (true=grzanie)
bool relayWasAbove=false;     // czy temp byla powyzej setpointu

unsigned long tP=0,tD=0,tR=0;
unsigned long rampT0=0;   // czas startu rampy (do soft startu feed-forward)
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

// Ostatnia (temp,ramp) dla ktorej wczytano profil - do wykrycia zmiany celu
// w trakcie dzialania AUTO (patrz ldProfIfChanged nizej).
float ldProfSpT=-9999,ldProfRU=-9999;

void ldProf(float temp,float ramp){
  int ti0=0,ti1=0,ri0=0,ri1=0;
  for(int i=0;i<PT_N-1;i++) if(temp>=PT[i]&&temp<=PT[i+1]){ti0=i;ti1=i+1;break;}
  if(temp<PT[0]){ti0=ti1=0;}if(temp>PT[PT_N-1]){ti0=ti1=PT_N-1;}
  for(int i=0;i<PR_N-1;i++) if(ramp>=PR[i]&&ramp<=PR[i+1]){ri0=i;ri1=i+1;break;}
  if(ramp<PR[0]){ri0=ri1=0;}if(ramp>PR[PR_N-1]){ri0=ri1=PR_N-1;}
  float wt=(ti1!=ti0)?(temp-PT[ti0])/(PT[ti1]-PT[ti0]):0.5f;
  float wr=(ri1!=ri0)?(ramp-PR[ri0])/(PR[ri1]-PR[ri0]):0.5f;
  float kph=0,kih=0,kdh=0,kpc=0,kic=0,kdc=0;float wsum=0;int cnt=0;
  auto add=[&](int ti,int ri,float w){
    Prof&p=prof[pi_(ti,ri)];
    if(p.valid){kph+=p.Kp_h*w;kih+=p.Ki_h*w;kdh+=p.Kd_h*w;
                kpc+=p.Kp_c*w;kic+=p.Ki_c*w;kdc+=p.Kd_c*w;wsum+=w;cnt++;}
  };
  add(ti0,ri0,(1-wt)*(1-wr));add(ti0,ri1,(1-wt)*wr);
  add(ti1,ri0,wt*(1-wr));add(ti1,ri1,wt*wr);
  if(cnt>0 && wsum>0.001f){
    // KLUCZOWE: przeskaluj przez sume wag waznych punktow.
    // Gdy czesc punktow niewykalibrowana, ich wagi przepadly - bez
    // tego przeskalowania wynik bylby zanizony (za slabe sterowanie).
    kph/=wsum;kih/=wsum;kdh/=wsum;kpc/=wsum;kic/=wsum;kdc/=wsum;
    Kp_h=constrain(kph,KP_MIN,KP_MAX);Ki_h=constrain(kih,KI_MIN,KI_MAX);Kd_h=constrain(kdh,KD_MIN,KD_MAX);
    Kp_c=constrain(kpc,KP_MIN,KP_MAX);Ki_c=constrain(kic,KI_MIN,KI_MAX);Kd_c=constrain(kdc,KD_MIN,KD_MAX);
    if(htg){Kp=Kp_h;Ki=Ki_h;Kd=Kd_h;}else{Kp=Kp_c;Ki=Ki_c;Kd=Kd_c;}
    Serial.print("Prof: Kp=");Serial.print(Kp,1);Serial.print(" Ki=");Serial.print(Ki,2);Serial.print(" Kd=");Serial.println(Kd,2);
  }
  ldProfSpT=temp;ldProfRU=ramp;
}

// ldProf() wczesniej byl wolany TYLKO raz, w momencie startu AUTO - jesli
// pozniej user zmienil TARGET (suwak SP, zawsze aktywny) w trakcie dzialania,
// Kp/Ki/Kd zostawaly ze STAREGO celu, calkowicie ignorujac tabele kalibracji
// dla nowej temperatury. To wola sie z kazdego throttlowanego cyklu AUTO -
// jesli cel (spT) lub rampa (rU) zmienily sie od ostatniego wczytania o
// wiecej niz 0.5, przeladuj interpolowany profil. Uwaga: to NIE jest pelny
// "gain scheduling" po biezacej temperaturze w trakcie rampy (celowo - zeby
// nie wprowadzac skokowych zmian Kp/Kd w trakcie samej rampy, co mogloby
// zaburzyc dopiero co ustabilizowane sledzenie), tylko reakcja na wyrazna
// zmiane CELU przez uzytkownika.
void ldProfIfChanged(){
  if(!calDone) return;
  if(fabs(spT-ldProfSpT)>0.5f || fabs(rU-ldProfRU)>0.5f) ldProf(spT,rU);
}

void savF(){FD fd;fd.ver=FLASH_VER;fd.cal=calDone;for(int i=0;i<P_TOT;i++) fd.p[i]=prof[i];fd.ru=rU;fd.rd=rD;fd.tm=tMax;fd.polSet=polSet;fd.polSw=polSw;fd.calMin=cTmn;fd.calMax=cTmx;pidFlash.write(fd);Serial.println("Flash: zapisano.");}
void ldF(){
  FD fd;pidFlash.read(fd);
  if(fd.ver!=FLASH_VER){
    // Stary/inny layout (np. przed zmiana siatki ramp PR_N) - bajty NIE
    // pasuja do aktualnego struct (przesuniete offsety), czytanie ich jako
    // biezace pola dawaloby smieci (losowe Kp/Ki/Kd, polSw, zakres
    // kalibracji). Ignorujemy CALA zawartosc - startujemy jak na czystym
    // urzadzeniu (kalibracja i polaryzacja do zrobienia od nowa).
    Serial.println("Flash: stary format (zmieniona siatka) - ignoruje, potrzebna nowa kalibracja.");
    return;
  }
  if(fd.cal){calDone=true;for(int i=0;i<P_TOT;i++) prof[i]=fd.p[i];rU=fd.ru;rD=fd.rd;tMax=fd.tm;Serial.println("Flash: wczytano.");}
  if(fd.polSet){polSet=true;polSw=fd.polSw;}
  if(fd.calMin>=0&&fd.calMin<fd.calMax&&fd.calMax<=115){cTmn=fd.calMin;cTmx=fd.calMax;}
}
void savePol(){FD fd;pidFlash.read(fd);fd.ver=FLASH_VER;fd.polSet=true;fd.polSw=polSw;pidFlash.write(fd);}
void rst(){calDone=false;for(int i=0;i<P_TOT;i++) prof[i]={10,0.3f,0.8f,10,0.3f,0.3f,false};Kp_h=Kp_c=Kp=10;Ki_h=Ki_c=Ki=0.3f;Kd_h=0.8f;Kd_c=Kd=0.3f;rU=rD=2;tMax=TEMP_MAX_DEF;ig=0;dInit=false;Serial.println("Reset.");}

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
  // Pelna moc bez limitu (bezpieczny zasilacz 5V/3A)
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
  uint8_t f=tc.readFault();
  if(f){
    tcE=true;
    if(!tcErrSent){Serial.print("ERR:code=1,bits=0x");Serial.print(f,HEX);Serial.println(",active=1");tcErrSent=true;}
    return lT;
  }
  if(tcErrSent){Serial.println("ERR:code=1,active=0");tcErrSent=false;}
  tcE=false;float raw=tc.readThermocoupleTemperature();
  // Ochrona przed NaN i bezsensownymi wartosciami
  if(isnan(raw)||raw<-50.0f||raw>200.0f){
    tcE=true;
    if(!tcRangeErrSent){Serial.print("ERR:code=2,val=");Serial.print(isnan(raw)?9999.0f:raw,1);Serial.println(",active=1");tcRangeErrSent=true;}
    return lT;
  }
  if(tcRangeErrSent){Serial.println("ERR:code=2,active=0");tcRangeErrSent=false;}
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
  ig=0;dInit=false;
  Serial.print("ST START SP=");Serial.print(spT,1);Serial.print(" R=");Serial.println(rU,1);
}
void stStop(){
  stOn=false;stDone=true;
  Kp_h=stBKpH;Ki_h=stBKiH;Kd_h=stBKdH;
  Kp_c=stBKpC;Ki_c=stBKiC;Kd_c=stBKdC;
  if(htg){Kp=Kp_h;Ki=Ki_h;Kd=Kd_h;}else{Kp=Kp_c;Ki=Ki_c;Kd=Kd_c;}
  int idx=pi_(nTi(spT),nRi(htg?rU:rD));
  prof[idx]={Kp_h,Ki_h,Kd_h,Kp_c,Ki_c,Kd_c,true};
  calDone=true;ig=0;dInit=false;
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

// ── Czysty PID (uproszczony, zoptymalizowany pod gladka linie) ────
// Ta funkcja byla kilka rund temu przeglądana i poprawiana pod katem
// konkretnych objawow (drganie, ff). Ponizsza wersja dodatkowo wciela DWIE
// klasyczne, bardzo dobrze sprawdzone techniki z Arduino-PID-Library Bretta
// Beauregarda (najpopularniejsza, uzywana od 2010 biblioteka PID na Arduino,
// https://github.com/br3ttb/Arduino-PID-Library - patrz tez jej rozwiniecie
// QuickPID) - a NIE podmiane calego PIDu na gotowa biblioteke, bo ta nie zna
// naszej rampy (spA), dwoch osobnych zestawow Kp/Ki/Kd (grzanie/chlodzenie)
// ani interpolowanej tabeli profili z kalibracji - musialaby i tak zostac
// dookola owinieta ta sama logika.
int compPID(float temp){
  float dt=PID_DT_MS/1000.0f,err=spA-temp;

  bool atT=(fabs(spA-spT)<0.5f);

  // ── CALKA w JEDNOSTKACH WYJSCIA (anti-windup jak w Arduino-PID-Library) ──
  // Beauregard: outputSum+=ki*error; potem klamrowanie outputSum do
  // outMin/outMax (PRAWDZIWEGO zakresu aktuatora), a nie do jakiejs osobnej
  // stalej. Wczesniej bylo tu: calkuj surowy blad (ig+=err*dt), pomnoz przez
  // Ki DOPIERO na koncu, a limit calki to sztywna stala INTEGRAL_MAX
  // (400) niezalezna od Ki. Problem: po kalibracji/self-tune Ki bywa bardzo
  // rozne (rzedy wielkosci) - ta sama stala 400 raz pozwalala na spory
  // windup, raz byla bez sensu ciasna. Calkujac OD RAZU w jednostkach
  // wyjscia i klamrujac do realnego 0..PWM_MAX (grzanie) / -PWM_MAX..0
  // (chlodzenie), limit automatycznie skaluje sie z Ki - dokladnie to, co
  // "prawdziwy" anti-windup ma robic (calka nigdy nie moze sama pchac
  // wyjscia poza to, co aktuator i tak może dac).
  // igScale=0.5 na rampie: FF (nizej) juz robi wiekszosc roboty; pelny
  // autorytet calki podczas rampy potrafil nabic "dziure" tuz przy celu.
  float igScale = atT ? 1.0f : 0.5f;
  ig += Ki*err*dt;
  if(htg) ig=constrain(ig, 0.0f, (float)PWM_MAX*igScale);
  else    ig=constrain(ig, -(float)PWM_MAX*igScale, 0.0f);

  // ── POCHODNA z POMIARU (derivative on measurement) + FILTR ──────────
  // Druga technika z Arduino-PID-Library: pochodna liczona z POMIARU
  // (temp), NIE z BLEDU (err). Sam pomiar nigdy nie skacze skokowo -
  // w przeciwienstwie do bledu, ktory moze skoczyc przy kazdej zmianie
  // SP/rampy albo przeladowaniu profilu (ldProfIfChanged w trakcie AUTO).
  // Liczac pochodna z bledu (jak wczesniej: dRaw=(err-pe)/dt), taki skok
  // od razu wchodzil w Kd jako ostry, jednorazowy impuls mocy
  // ("derivative kick") - klasyczny, dobrze udokumentowany blad w prostych
  // implementacjach PID.
  // dInit chroni PIERWSZY cykl po kazdym resecie (ig=0;dInit=false) - bez
  // tego peT (stara/startowa wartosc) dawalby fikcyjny wielki skok
  // pochodnej na jedna probke zaraz po starcie/zmianie kierunku/wyjsciu
  // z kalibracji.
  float dMeas;
  if(!dInit){ peT=temp; dInit=true; dMeas=0.0f; }
  else      { dMeas = -(temp-peT)/dt; peT=temp; }  // -d(temp)/dt ~ d(err)/dt gdy SP nie skacze
  dFilt = dFilt + 0.15f*(dMeas - dFilt);   // filtrowana pochodna

  // ── FEED-FORWARD rampy ──────────────────────────────
  // Wczesniej FF_GAIN byl zdefiniowany ale NIGDZIE nieuzywany (naglowek
  // pliku mowil wprost "Czysty PID (bez FF)") - caly ciezar podazania za
  // rosnaca rampa spoczywal na reaktywnym PID, co przy duzym Kd = drganie
  // zamiast plynnego "pchania w gore". FF dodaje stala, NIESZUMIACA
  // skladowa proporcjonalna do zadanej predkosci rampy - PID musi juz
  // tylko lekko dokorygowywac, a nie w 100% reaktywnie goni cel. Aktywny
  // TYLKO w trakcie rampy (nie przy juz osiagnietym celu, zeby nie psul
  // precyzyjnego trzymania temperatury w miejscu).
  // UWAGA: kierunek FF to (spT>spA) - kierunek RAMPY (dokad zmierza spA),
  // NIE htg (kierunek REALNIE wypuszczanej mocy) - to dwie rozne rzeczy,
  // patrz komentarz przy przelaczaniu kierunku nizej.
  float ff = atT ? 0.0f : FF_GAIN*((spT>spA)?rU:-rD);

  // Wyjscie PID (feed-forward + P + calka-w-jednostkach-wyjscia + D),
  // policzone JESZCZE bez twardego obciecia do 0..PWM_MAX - potrzebne
  // surowe (nieprzyciete) do decyzji o kierunku ponizej.
  float out=ff + Kp*err + ig + Kd*dFilt;

  // ── KIERUNEK: grzanie = zwiekszamy PWM, "chlodzenie" = zmniejszamy PWM
  //    AZ do momentu w ktorym musimy odwrocic polaryzacje ────────────
  // Metodyka (na zyczenie, bez zadnego sztucznego opoznienia): dopoki
  // grzejemy, PID normalnie rosnie/maleje w gore/w dol PWM. Gdy trzeba
  // mniej mocy, PWM po prostu spada w kierunku zera - to JEST "chlodzenie
  // samym schodzeniem z PWM" (oddawanie ciepla do otoczenia), zaden osobny
  // mechanizm nie jest do tego potrzebny, bo to sie dzieje samo przez
  // normalny spadek wyjscia PID. Aktywne chlodzenie (ujemny PWM, odwrocenie
  // Peltiera) wlacza sie DOKLADNIE w momencie, w ktorym SUROWE (nieprzyciete)
  // wyjscie PID faktycznie przechodzi na ujemne - czyli nawet PWM=0 juz nie
  // wystarcza i regulator "chce" isc jeszcze dalej. To jest fizyczna
  // konieczna granica ("musimy odwrocic polaryzacje"), bez zadnej stalej
  // typu limit czasu/margines - poprzednia wersja z 20s opoznieniem zostaje
  // usunieta na wyrazne zyczenie.
  // Przy przelaczeniu: reset calki/pochodnej (inny profil = inna skala) i
  // podmiana Kp/Ki/Kd na drugi (juz skalibrowany osobno) zestaw.
  if(htg && out<0.0f){
    htg=false; ig=0; dInit=false; Kp=Kp_c;Ki=Ki_c;Kd=Kd_c;
  } else if(!htg && out>0.0f){
    htg=true; ig=0; dInit=false; Kp=Kp_h;Ki=Ki_h;Kd=Kd_h;
  }

  // ── TWARDA JEDNOKIERUNKOWOSC ─────────────────────────
  // Grzanie: tylko dodatni PWM. Chlodzenie: tylko ujemny. Bez mieszania.
  if(htg) out=constrain(out,0.0f,(float)PWM_MAX);
  else    out=constrain(out,-(float)PWM_MAX,0.0f);

  // ── FILTR PWM (wygladzenie mocy) ─────────────────────
  // Zamiast slew rate - filtr dolnoprzepustowy na wyjsciu PWM.
  // Wygladza skoki mocy = gladki przyrost = prosta linia temperatury.
  // alfa=0.4: kompromis miedzy gladkoscia a responsywnoscia.
  // Reset (pwmFilt=out) gdy wlasnie ruszyl cykl - bez naleciaosci.
  pwmFilt = pwmFilt + 0.4f*(out - pwmFilt);

  return (int)pwmFilt;
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
void stCalS(){
  // stOn=false: jesli self-tune byl aktywny gdy wchodzimy do kalibracji,
  // zostawal wlaczony "w tle" (runST tylko wychodzi wczesnie bo sys!=AUTO,
  // ale stOn nigdy sie nie czyscil) - po powrocie do AUTO bez wyraznego
  // SELFTUNESTOP self-tune nagle sam wskakiwal z powrotem i zaczynal
  // dostrajac Kp/Ki/Kd bez pytania.
  sys=CAL;cPh=-1;cSt="Ustaw zakres";stOn=false;
}
void stCalR(){
  bldCP();cTi=cRi=cPh=cIt=0;cPT=millis();cBH=cBC=999;
  // Start od wartosci bazowych
  Kp_h=KP_BASE;Ki_h=KI_BASE;Kd_h=KD_BASE_H;Kp_c=KP_BASE;Ki_c=KI_BASE;Kd_c=KD_BASE_C;
  cKpH=Kp_h;cKiH=Ki_h;cKdH=Kd_h;cKpC=Kp_c;cKiC=Ki_c;cKdC=Kd_c;
  for(int i=0;i<CH;i++){cEH[i]=cPwH[i]=0;}cHI=0;cLI=0;ig=0;dInit=false;
  spT=cTP[0];spA=lT;rU=rD=RAMP_MAX;
  int tot=cTN;  // relay: tylko temperatury (nie temp*rampa)
  char b[24];sprintf(b,"Start 1/%d",tot);cSt=String(b);
  Serial.println("=== KAL. RELAY START ===");
  // Plan: CALPLAN ma temps; ramps=relay (jeden test na temperature)
  Serial.print("CALPLAN:");Serial.print(tot);
  Serial.print(",temps=");
  for(int i=0;i<cTN;i++){Serial.print(cTP[i],0);if(i<cTN-1)Serial.print("/");}
  Serial.print(",ramps=relay");
  Serial.println();
}
void savCP(){
  // Relay: profil zalezy tylko od TEMPERATURY (nie od rampy).
  // Wypelnij WSZYSTKIE rampy tej temperatury tym samym profilem.
  //
  // WAZNE: valid=cLastOk, NIE na sztywno true. Wczesniej kazdy punkt -
  // rowniez ten gdzie relay test sie nie udal i uzylismy wartosci
  // bazowych ("RELAY FAIL - bazowe") - byl zapisywany jako valid=true.
  // ldProf()/add() ufa flagi valid bez zadnej dodatkowej weryfikacji, wiec
  // taki "fallbackowy" punkt byl w interpolacji nie do odroznienia od
  // naprawde skalibrowanego - agresywne wartosci bazowe (Kp=10 itd., dobrane
  // "na oko" jako start kalibracji, nie jako cos co ma dobrze trzymac
  // temperature) potrafily wywolac trwala oscylacje przy trzymaniu SP w
  // AUTO w okolicy takiego punktu (lub jego interpolowanego sasiedztwa).
  // Komentarz w ldProf() przy przeskalowaniu przez wsum zaklada wlasnie, ze
  // niekalibrowane punkty maja valid=false i wypadaja z interpolacji - to
  // przywraca ten zamierzony efekt.
  int ti=nTi(cTP[cTi]);
  for(int ri=0;ri<PR_N;ri++){
    int idx=pi_(ti,ri);
    prof[idx]={cKpH,cKiH,cKdH,cKpC,cKiC,cKdC,cLastOk};
  }
  Serial.print("Prof T=");Serial.print(cTP[cTi],0);Serial.print(" (wszystkie rampy) Kp=");Serial.print(cKpH,1);
  Serial.println(cLastOk?"":" [FALLBACK-bazowe, valid=0]");
}
// startRampPrep(): przed testem KAZDEJ rampy trzeba sie cofnac ponizej celu
// (RAMP_TEST_DROP), bo updRamp() nic nie robi gdy spA juz ==spT - test
// rampowania musialby "jechac" z zerowego dystansu. Startuje ZAWSZE od
// bazowego profilu z relay (cKpH itd.), nie od wyniku poprzedniej rampy -
// powtarzalnosc, kazdy wynik niezalezny od kolejnosci testowania ramp.
void startRampPrep(){
  cPh=4;cPT=millis();
  float dropTo=cTP[cTi]-RAMP_TEST_DROP;
  if(dropTo<TEMP_MIN_C+2.0f) dropTo=TEMP_MIN_C+2.0f;  // bezpiecznik - nie schodz ponizej sensownego zakresu
  spT=dropTo;spA=lT;rU=rD=RAMP_MAX;  // pelna predkosc cofniecia, jak dojscie do temp bazowej
  Kp_h=cKpH;Ki_h=cKiH;Kd_h=cKdH;Kp_c=cKpC;Ki_c=cKiC;Kd_c=cKdC;
  ig=0;dInit=false;
  char b[32];sprintf(b,"Ramp %d/%d: cofam sie",cRi+1,calRampN);cSt=String(b);
}
void finishTempPoint(){
  // Wywolywane PO relay+savCP i PO wszystkich testach rampowania dla tej
  // temperatury (albo od razu jesli calRampN==0) - przejdz do kolejnej
  // temperatury albo zakoncz kalibracje.
  cTi++;
  int tot=cTN,done=cTi;
  if(cTi>=cTN){calDone=true;savF();sys=MAN;stpPel();char b[24];sprintf(b,"DONE %d/%d",tot,tot);cSt=String(b);Serial.println("=== KAL. ZAKONCZONA ===");return;}
  cPh=0;cPT=millis();
  Kp_h=KP_BASE;Ki_h=KI_BASE;Kd_h=KD_BASE_H;Kp_c=KP_BASE;Ki_c=KI_BASE;Kd_c=KD_BASE_C;
  spT=cTP[cTi];rU=rD=RAMP_MAX;spA=lT;ig=0;dInit=false;
  char b[24];sprintf(b,"Temp %d/%d",done+1,tot);cSt=String(b);
}
// ── KALIBRACJA ────────────────────────────────────────────
// Dla kazdej temperatury:
//   Faza 0: dochodzenie do temp bazowej z pelna moca
//   Faza 1: stabilizacja na temp bazowej (15s)
//   RELAY FEEDBACK AUTOTUNING (standard przemyslowy Astrom-Hagglund)
//   Faza 2: RELAY - wymus oscylacje przekaznikiem, zmierz Ku i Tu
//   Faza 3: oblicz Kp/Ki/Kd (Tyreus-Luyben), zapisz bazowy profil (savCP)
//   Potem, DLA KAZDEJ rampy z calRamps[] (test rampowania, patrz wyzej):
//   Faza 4: cofniecie sie RAMP_TEST_DROP ponizej celu
//   Faza 5: grzanie z testowanym rampem do celu, sledzenie i dostrajanie
//   Faza 6: zapisz wynik tej rampy, nastepna rampa albo nastepna temperatura
void runCal(float temp){
  if(sys!=CAL||cPh==-1) return;
  unsigned long now=millis(),el=now-cPT;
  float err=spA-temp,ae=abs(err);

  if(cPh==0){
    // Dochodzenie do temperatury bazowej - PELNA predkosc (szybki dojazd)
    rU=rD=RAMP_MAX;
    if(now-tR>=DT_R){tR=now;updRamp();}
    // WAZNE: compPID() zaklada stale dt=PID_DT_MS (100ms) miedzy wywolaniami
    // (patrz jego komentarz "float dt=PID_DT_MS/1000.0f"). runCal() jest
    // wolane z KAZDEJ iteracji loop() (bez throttlingu), a loop() na tym
    // procku potrafi obrocic sie tysiace razy na 100ms - bez tej bramki
    // compPID() bylby wolany tysiace razy "na sucho" z tym samym temp/err,
    // za kazdym razem doliczajac 100ms do calki (ig+=err*dt) tak jakby
    // naprawde minelo 100ms. Efekt: ig natychmiast saturuje do limitu
    // anti-windup zamiast narastac w realnym czasie - integrator przestaje
    // dzialac jak integrator. Throttlujemy do tego samego tP/PID_DT_MS co
    // reszta systemu (AUTO/FREEZE nizej w loop()).
    if(now-tP>=PID_DT_MS) setPwr(compPID(temp));
    char b[32];sprintf(b,"->%.0fC T=%.1f",cTP[cTi],temp);
    cSt=String(b);
    // Status dla aplikacji co 500ms (zeby okno postepu nie milczalo)
    if(now-cLI>=500){
      cLI=now;
      int tot=cTN,done=cTi+1;
      Serial.print("CALSTAT:");Serial.print(done);Serial.print("/");
      Serial.print(tot);Serial.print(",T=");Serial.print(cTP[cTi],0);
      Serial.println(",R=heating");
    }
    if(ae<2.0f||el>CA){
      cPh=1;cPT=now;cSt="Stabilizing...";
      ig=0;dInit=false;cLI=now;
    }
  }
  else if(cPh==1){
    // Stabilizacja na temperaturze bazowej (patrz komentarz w cPh==0 -
    // ta sama bramka throttlingu, z tego samego powodu).
    if(now-tP>=PID_DT_MS) setPwr(compPID(temp));
    cSt="Stabil "+String((CS-el)/1000)+"s";
    if(now-cLI>=500){
      cLI=now;
      int tot=cTN,done=cTi+1;
      Serial.print("CALSTAT:");Serial.print(done);Serial.print("/");
      Serial.print(tot);Serial.print(",T=");Serial.print(cTP[cTi],0);
      Serial.println(",R=stabil");
    }
    if(el>CS){
      // Przejdz do RELAY testu - inicjalizuj pomiar oscylacji
      cPh=2;cPT=now;
      spT=cTP[cTi];spA=cTP[cTi];  // setpoint = temperatura bazowa (staly)
      relayPeakHi=-999;relayPeakLo=999;
      for(int i=0;i<RELAY_CYCLES;i++){relayAmps[i]=relayPers[i]=0;}
      relayCycN=0;
      relayAmpCur=RELAY_AMP;  // zawsze startuj od lagodnej amplitudy
      relayTcross=now;
      relayWasAbove=(temp>spT);
      relayState=(temp<spT);
      cLI=now;  // dla throttlingu logow
      ig=0;dInit=false;
      cSt="Relay test...";
    }
  }
  else if(cPh==2){
    // ── RELAY FEEDBACK: wymus oscylacje, mierz Ku i Tu ──
    // Przekaznik z histereza: ponizej setpointu -> grzanie, powyzej -> chlodzenie.
    float sp=cTP[cTi];
    bool above=(temp > sp+RELAY_HYST);
    bool below=(temp < sp-RELAY_HYST);
    if(below) relayState=true;
    else if(above) relayState=false;
    setPwr(relayState ? (int)relayAmpCur : -(int)relayAmpCur);

    // Sledz szczyty w biezacym cyklu
    if(temp>relayPeakHi) relayPeakHi=temp;
    if(temp<relayPeakLo) relayPeakLo=temp;

    // Przejscie przez setpoint z dolu do gory = koniec cyklu.
    // Zbyt szybkie przejscia (< RELAY_MIN_PERIOD_MS) to niemal na pewno szum
    // odczytu termopary / zaklocenia PWM sterownika, nie prawdziwa oscylacja
    // termiczna - CALKOWICIE je ignorujemy (nie resetujemy szczytow ani
    // relayTcross), zeby nie porozbijac prawdziwego, wolniejszego cyklu na
    // fragmenty i nie wliczyc smieciowego okresu do obliczen Tu/Ku.
    bool nowAbove=(temp>sp);
    if(nowAbove && !relayWasAbove){
      unsigned long per=now-relayTcross;
      if(per>=RELAY_MIN_PERIOD_MS){
        if(relayCycN>0){
          // Zapisz amplitude i okres TEGO cyklu do tablic (bierzemy pozniej
          // tylko OSTATNIE ustalone cykle, bo pierwsze moga narastac).
          float amp=(relayPeakHi-relayPeakLo)/2.0f;
          int slot=(relayCycN-1)%RELAY_CYCLES;
          relayAmps[slot]=amp;
          relayPers[slot]=(float)per;
        }
        relayTcross=now;
        relayPeakHi=-999;relayPeakLo=999;
        relayCycN++;
      }
      // else: zbyt szybkie, potraktuj jako szum w trakcie tej samej polowy
      // cyklu - relayTcross/relayPeak* NIE sa resetowane.
    }
    relayWasAbove=nowAbove;

    // Eskalacja amplitudy: jesli od RELAY_ESCALATE_MS nie zlapalismy ani
    // jednego prawdziwego przejscia, uklad prawdopodobnie WISI po jednej
    // stronie setpointu (za blisko otoczenia na dole zakresu / za blisko
    // granicy mocy grzania na gorze) i lagodna amplituda nigdy go nie
    // przepchnie na druga strone. Podbij moc i daj nowej amplitudzie wlasne
    // RELAY_ESCALATE_MS zanim zdecydujemy ze i tak sie nie da.
    if(now-relayTcross>RELAY_ESCALATE_MS && relayAmpCur<RELAY_AMP_MAX){
      relayAmpCur+=RELAY_AMP_STEP;
      if(relayAmpCur>RELAY_AMP_MAX) relayAmpCur=RELAY_AMP_MAX;
      relayTcross=now;
      relayPeakHi=-999;relayPeakLo=999;
      Serial.print("RELAY escalate T=");Serial.print(sp,0);
      Serial.print(" amp=");Serial.println(relayAmpCur,0);
    }

    char b[40];sprintf(b,"Relay %d/%d @%d%%",relayCycN,RELAY_EXIT_CYCLES,(int)(relayAmpCur*100/PWM_MAX));
    cSt=String(b);

    // Log + status TYLKO co 500ms (nie zalewaj portu - inaczej aplikacja
    // gubi komunikaty CALPLAN/CALSTAT i okno postepu jest puste).
    if(now-cLI>=500){
      cLI=now;
      int tot=cTN,done=cTi+1;
      Serial.print("CALSTAT:");Serial.print(done);Serial.print("/");
      Serial.print(tot);Serial.print(",T=");Serial.print(sp,0);
      Serial.println(",R=relay");
      Serial.print(now/1000.0f,1);Serial.print(",");Serial.print(temp,2);Serial.print(",");
      Serial.print(sp,2);Serial.print(",");Serial.print(sp,2);Serial.print(",");
      Serial.print(lPwm);Serial.print(",");Serial.print(Kp,3);Serial.print(",");
      Serial.print(Ki,4);Serial.print(",");Serial.print(Kd,3);
      Serial.print(",CAL-");Serial.println(done);
    }

    if(relayCycN>=RELAY_EXIT_CYCLES || el>RELAY_MAX_MS){
      cPh=3;cPT=now;
    }
  }
  else if(cPh==3){
    // ── OBLICZ Kp/Ki/Kd z pomiaru (Tyreus-Luyben) ──
    setPwr(0);
    // Wez OSTATNIE cykle (ustalone), policz srednia z tablic.
    // Pierwsze cykle moga narastac - ostatnie sa miarodajne.
    int valid=0; float ampSum=0, perSum=0;
    for(int i=0;i<RELAY_CYCLES;i++){
      if(relayAmps[i]>0.01f && relayPers[i]>0){
        ampSum+=relayAmps[i]; perSum+=relayPers[i]; valid++;
      }
    }
    if(valid>=2 && ampSum>0.01f){
      float aAvg=ampSum/valid;             // srednia amplituda [C]
      float Tu=(perSum/valid)/1000.0f;     // sredni okres [s]
      // UWAGA: uzywamy relayAmpCur (aktualnej, mozliwe ze eskalowanej mocy
      // przekaznika), NIE stalej RELAY_AMP - inaczej Ku wyszloby zle jesli
      // ostatnie (uzyte do usredniania) cykle zlapaly sie dopiero po
      // podbiciu amplitudy.
      float Ku=(4.0f*relayAmpCur)/(3.14159f*aAvg);
      float Kp_new=Ku/2.2f;
      float Ti=2.2f*Tu;
      float Td=Tu/6.3f;
      float Ki_new=Kp_new/Ti;
      float Kd_new=Kp_new*Td;
      Kp_new=constrain(Kp_new,KP_MIN,KP_MAX);
      Ki_new=constrain(Ki_new,KI_MIN,KI_MAX);
      Kd_new=constrain(Kd_new,KD_MIN,KD_MAX);
      cKpH=Kp_new;cKiH=Ki_new;cKdH=Kd_new;
      cKpC=Kp_new;cKiC=Ki_new;cKdC=Kd_new;
      cLastOk=true;
      Serial.print("RELAY T=");Serial.print(cTP[cTi],0);
      Serial.print(" a=");Serial.print(aAvg,1);Serial.print(" Tu=");Serial.print(Tu,1);
      Serial.print(" Ku=");Serial.print(Ku,1);
      Serial.print(" Kp=");Serial.print(Kp_new,2);Serial.print(" Ki=");Serial.print(Ki_new,3);
      Serial.print(" Kd=");Serial.println(Kd_new,2);
    } else {
      cKpH=KP_BASE;cKiH=KI_BASE;cKdH=KD_BASE_H;
      cKpC=KP_BASE;cKiC=KI_BASE;cKdC=KD_BASE_C;
      cLastOk=false;
      Serial.println("RELAY FAIL - bazowe");
      // Sygnal dla aplikacji PC (masz surowa konsole czy nie - to trafi do GUI):
      // ten punkt NIE zostal naprawde skalibrowany, uzyto wartosci bazowych.
      // amp=... to amplituda przy ktorej test sie poddal (jesli to juz
      // RELAY_AMP_MAX, to nawet ~55% mocy nie przepchnelo ukladu przez
      // setpoint w obie strony - warto sprawdzic czy ta temperatura jest w
      // ogole fizycznie osiagalna/utrzymywalna na tym sprzecie).
      Serial.print("CALWARN:T=");Serial.print(cTP[cTi],0);
      Serial.print(",cycles=");Serial.print(relayCycN);
      Serial.print(",amp=");Serial.print((int)relayAmpCur);
      Serial.println(",relay_fail");
    }
    // wyczysc tablice na nastepna temperature
    for(int i=0;i<RELAY_CYCLES;i++){relayAmps[i]=relayPers[i]=0;}
    savCP();  // bazowy profil z relay -> wszystkie rampy tej temperatury (fallback zanim/jesli testy rampowania je dopracuja)
    if(calRampN>0){ cRi=0; startRampPrep(); }
    else{ finishTempPoint(); }
  }
  else if(cPh==4){
    // ── Cofniecie sie przed testem tej rampy (patrz startRampPrep()) ──
    if(now-tR>=DT_R){tR=now;updRamp();}
    if(now-tP>=PID_DT_MS) setPwr(compPID(temp));  // throttling - patrz komentarz w cPh==0
    char b[32];sprintf(b,"Ramp %d/%d: ->%.0fC",cRi+1,calRampN,spT);cSt=String(b);
    if(now-cLI>=500){
      cLI=now;
      int tot=cTN,done=cTi+1;
      Serial.print("CALSTAT:");Serial.print(done);Serial.print("/");
      Serial.print(tot);Serial.print(",T=");Serial.print(cTP[cTi],0);
      Serial.print(",R=rampprep:");Serial.println(calRamps[cRi],0);
    }
    if(ae<2.0f||el>CA){
      // Gotowe do jazdy w gore - ustaw PRAWDZIWY cel i testowany rampe.
      cPh=5;cPT=now;cRT=now;
      spT=cTP[cTi];rU=rD=calRamps[cRi];
      cRampBestErr=999;cRampBestKpH=Kp_h;cRampBestKiH=Ki_h;cRampBestKdH=Kd_h;
      for(int i=0;i<CH;i++){cEH[i]=cPwH[i]=0;}cHI=0;
      ig=0;dInit=false;cLI=now;
    }
  }
  else if(cPh==5){
    // ── TEST ROMPOWANIA: grzej z testowanym rampem, sledz i dostrajaj ──
    if(now-tR>=DT_R){tR=now;updRamp();}
    if(now-tP>=PID_DT_MS) setPwr(compPID(temp));
    char b[32];sprintf(b,"Ramp %d/%d @%.0f",cRi+1,calRampN,calRamps[cRi]);cSt=String(b);
    // Co CI ms: identyczna heurystyka co self-tune (patrz runST()) - ale
    // TYLKO galaz grzania (ten test explicitnie tylko grzeje, patrz
    // komentarz przy #define RAMP_TEST_DROP wyzej). Jesli akurat htg==false
    // (chwilowe pasywne chlodzenie przy przeregulowaniu) - pomijamy analize
    // tego tyknienia, nie ma co dostrajac galezi ktora teraz nie steruje.
    if(now-cRT>=CI){
      cRT=now;
      if(htg){
        float e=spA-temp,ae2=fabs(e);
        cEH[cHI]=e;cPwH[cHI]=(float)lPwm;cHI=(cHI+1)%CH;
        int sc=0;for(int i=0;i<CH-1;i++){int a=i,b=(i+1)%CH;if(cEH[a]*cEH[b]<0)sc++;}
        bool osc=(sc>=3);
        int sat=0;for(int i=0;i<CH;i++) if(fabs(cPwH[i])>=PWM_MAX-5) sat++;
        bool satd=(sat>=CH-1);
        int pi2=(cHI-2+CH)%CH,ci2=(cHI-1+CH)%CH;
        float tr=fabs(cEH[ci2])-fabs(cEH[pi2]);
        bool im=(tr<-0.1f),wo=(tr>0.3f);
        if(osc){Kp_h=constrain(Kp_h*(1-ST_ADJ*1.5f),KP_MIN,KP_MAX);Kd_h=constrain(Kd_h*(1-ST_ADJ),KD_MIN,KD_MAX);Ki_h=constrain(Ki_h*(1-ST_ADJ*0.5f),KI_MIN,KI_MAX);ig*=0.5f;}
        else if(satd&&ae2>2){/* nasycone PWM - nic nie da sie wiecej wycisnac, czekaj */}
        else if(ae2>8&&!im){Kp_h=constrain(Kp_h*(1+ST_ADJ*2),KP_MIN,KP_MAX);}
        else if(ae2>3&&wo){Kp_h=constrain(Kp_h*(1+ST_ADJ),KP_MIN,KP_MAX);}
        else if(ae2>ST_DEAD&&im){Ki_h=constrain(Ki_h*(1+ST_ADJ*0.5f),KI_MIN,KI_MAX);}
        Kp=Kp_h;Ki=Ki_h;Kd=Kd_h;
        if(ae2<cRampBestErr){cRampBestErr=ae2;cRampBestKpH=Kp_h;cRampBestKiH=Ki_h;cRampBestKdH=Kd_h;}
      }
    }
    if(now-cLI>=500){
      cLI=now;
      int tot=cTN,done=cTi+1;
      Serial.print("CALSTAT:");Serial.print(done);Serial.print("/");
      Serial.print(tot);Serial.print(",T=");Serial.print(cTP[cTi],0);
      Serial.print(",R=ramptest:");Serial.println(calRamps[cRi],0);
    }
    if(el>=CT){ cPh=6; cPT=now; }
  }
  else if(cPh==6){
    // ── Zapisz wynik tej rampy (tylko galaz grzania), nastepna rampa/temp ──
    int ti=nTi(cTP[cTi]),ri=nRi(calRamps[cRi]);
    int idx=pi_(ti,ri);
    bool ok=(cRampBestErr<RAMP_TEST_OK_ERR);
    if(ok){
      prof[idx].Kp_h=cRampBestKpH;prof[idx].Ki_h=cRampBestKiH;prof[idx].Kd_h=cRampBestKdH;
      prof[idx].valid=true;
      Serial.print("RAMPTEST T=");Serial.print(cTP[cTi],0);Serial.print(" R=");Serial.print(calRamps[cRi],0);
      Serial.print(" err=");Serial.print(cRampBestErr,2);
      Serial.print(" Kp=");Serial.print(cRampBestKpH,2);Serial.print(" Ki=");Serial.print(cRampBestKiH,3);
      Serial.print(" Kd=");Serial.println(cRampBestKdH,2);
    } else {
      // Nie udalo sie zejsc ponizej progu - zostaw bazowy profil z relay
      // (juz tam jest, savCP() go wpisal wczesniej) i ostrzez apke PC.
      Serial.print("CALWARN:T=");Serial.print(cTP[cTi],0);
      Serial.print(",R=");Serial.print(calRamps[cRi],0);
      Serial.print(",err=");Serial.print(cRampBestErr,2);
      Serial.println(",ramp_track_fail");
    }
    cRi++;
    if(cRi<calRampN){ startRampPrep(); }
    else{ finishTempPoint(); }
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
        ig=0;dInit=false;slT=0;tR=millis();
        if(calDone) ldProf(spT,rU);
        // BEZ auto-start self-tune – uzywaj zapisanych parametrow
        // Self-tune trzeba uruchomic recznie z menu
        Serial.println("ON");
      } else if(sys==AUTO){
        stpPel();sys=MAN;stOn=false;
        fanOn=true; fanSpeed=100; fanApply();
        fanRunonActive=true; fanRunonT=millis();
        Serial.println("STOP");
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
  // Tryb relay: jeden test na temperature, wiec "total" to cTN, nie
  // cTN*calRampN (to bylo z poprzedniego trybu siatki temp x rampa i tu
  // zawsze liczylo prawie 0% postepu bo cRi juz nigdy sie nie zmienia).
  int tot=cTN,done=cTi;oled.drawStr(0,9,"CALIBRATION");
  int pg=(tot>0)?done*56/tot:0;oled.drawFrame(70,2,58,8);if(pg>0) oled.drawBox(71,3,min(pg,56),6);
  oled.drawHLine(0,12,128);
  // Temp duza (y=16-36)
  oled.setFont(u8g2_font_10x20_tf);oled.drawStr(0,36,(fts(temp,1)+"C").c_str());
  // Cel T - prawa strona, ponizej paska (y=28). Tryb relay nie ma pojecia
  // "rampy" (jeden test na temperature, wypelnia wszystkie rampy) - wczesniej
  // bylo tu "R%.0f" z calRamps[cRi], ale cRi nigdy sie nie zmienia w tym
  // trybie wiec zawsze pokazywalo pierwsza z listy (2C/min), mylnie sugerujac
  // ze akurat testujemy przy tej rampie.
  oled.setFont(u8g2_font_6x10_tf);
  if(cTi<cTN){char b[24];sprintf(b,"T%.0f",cTP[cTi]);
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
  if(!tc.begin()){Serial.println("ERROR: MAX31856!");Serial.println("ERR:code=4,msg=init_fail,active=1");}
  tc.setThermocoupleType(MAX31856_TCTYPE_K);
  // Druga termopara (pomiar dodatkowy) - osobny CS, ta sama magistrala SPI.
  // Zawsze inicjalizuj i zawsze probuj czytac - jesli modulu nie ma,
  // odczyt bedzie NAN i aplikacja pokaze kreski (poprawnie).
  tc2.begin();
  tc2.setThermocoupleType(MAX31856_TCTYPE_K);
  tc2OK=true;  // zawsze probuj czytac
  Serial.println("TC2 init done");
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

  // SP/RU/RD podczas CAL lub FREEZE: IGNORUJ. W obu trybach spT/rU/rD sa
  // zarzadzane WEWNETRZNIE (kalibracja: cel biezacego punktu; freeze: stala
  // temperatura docelowa galu) - apka PC ma suwaki TARGET/RATE zawsze
  // aktywne (mozna je ustawiac przed polaczeniem), wiec przypadkowe
  // przesuniecie suwaka w trakcie kalibracji/freeze wczesniej po cichu
  // psulo trwajacy pomiar / zaburzalo utrzymanie temperatury galu.
  if(key=="SP"){ if(sys!=CAL&&sys!=FREEZE) spT=constrain(fv,SP_MIN,SP_MAX); }
  else if(key=="RU"){ if(sys!=CAL&&sys!=FREEZE) rU=constrain(fv,RAMP_MIN,RAMP_MAX); }
  else if(key=="RD"){ if(sys!=CAL&&sys!=FREEZE) rD=constrain(fv,RAMP_MIN,RAMP_MAX); }
  else if(key=="TMAX"){ tMax=constrain(fv,TMAX_MIN,TMAX_MAX); }
  else if(key=="KP"){ Kp=constrain(fv,KP_MIN,KP_MAX); }
  else if(key=="KI"){ Ki=constrain(fv,KI_MIN,KI_MAX); }
  else if(key=="KD"){ Kd=constrain(fv,KD_MIN,KD_MAX); }
  else if(key=="OFFSET"){ calOffset=constrain(fv,-20.0f,20.0f); }
  else if(key=="START"){
    if(sys==MAN){
      sys=AUTO;spA=lT;ig=0;dInit=false;slT=0;tR=millis();
      dFilt=0;pwmFilt=0;   // reset filtrow - bez naleciaosci z poprzedniego cyklu
      rampT0=millis();
      if(calDone) ldProf(spT,rU);
      Serial.println("ON");
    }
  }
  else if(key=="STOP"){
    // Wylacz Peltier OD RAZU (bez schodzenia do niskich temp).
    // Wentylator ZAWSZE wlacza sie na 100% i chlodzi radiator przez FAN_RUNON_MS.
    stpPel(); sys=MAN; stOn=false;
    fanOn=true; fanSpeed=100; fanApply();   // auto-chlodzenie radiatora
    fanRunonActive=true; fanRunonT=millis();
    Serial.println("STOP");
  }
  else if(key=="ESTOP"){ wPwm(0);stpPel();sys=MAN;stOn=false;Serial.println("E-STOP"); }
  else if(key=="FREEZE"){
    // Tryb zamrazania galu - lagodne zejscie do FREEZE_TARGET i AKTYWNE utrzymanie.
    // Nie wylacza Peltiera (zapobiega odbiciu ciepla i ponownemu stopieniu galu).
    sys=FREEZE; spT=FREEZE_TARGET; spA=lT; ig=0; dInit=false; slT=0;
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
    // Przycisk "start kalibracji" w apce jest ZAWSZE klikalny (nawet w
    // trakcie trwajacej kalibracji) - bez tej bramki powtorne/przypadkowe
    // kliknieciecie po cichu restartowalo caly przebieg od zera, tracac
    // caly dotychczasowy postep bez ostrzezenia.
    if(sys==CAL){
      Serial.println("AUTOCAL ignored - juz trwa (uzyj AUTOCALSTOP zeby przerwac)");
    } else {
      // Pelna automatyczna kalibracja wszystkich profili
      sys=CAL; cPh=-1; stOn=false;  // patrz komentarz w stCalS()
      stCalR();  // start kalibracji od razu
      Serial.println("AUTOCAL START");
    }
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
  // Wentylatory dobiegowe: po STOP chodza FAN_RUNON_MS, potem auto-wylaczenie
  if(fanRunonActive){
    if(now-fanRunonT>=FAN_RUNON_MS){
      fanRunonActive=false; fanOn=false; fanApply();
      Serial.println("FAN runon done - off");
    }
  }
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
    // Odczyt drugiej termopary (pomiar dodatkowy, nie wplywa na regulacje)
    if(tc2OK){
      // Czytaj temperature niezaleznie od faultu - MAX31856 czesto zglasza
      // drobne faulty (np. zakres) ale temperatura jest poprawna.
      // Odrzucamy tylko ewidentnie zle odczyty (NAN, ekstremalne wartosci).
      float r2=tc2.readThermocoupleTemperature();
      if(!isnan(r2) && r2>-50 && r2<200) temp2=r2;
      else temp2=NAN;
    }
    if(temp>tMax&&sys!=MAN){stpPel();sys=MAN;stOn=false;if(fanOn){fanRunonActive=true;fanRunonT=now;}
      Serial.print("ERR:code=3,temp=");Serial.print(temp,1);Serial.print(",limit=");Serial.println(tMax,1);
      Serial.println("!!! TEMP MAX - STOP !!!");}
    switch(sys){
      case AUTO:
        ldProfIfChanged();  // przeladuj Kp/Ki/Kd jesli TARGET/RATE zmienily sie w locie
        if(temp<TEMP_MIN_C&&lPwm<0) stpPel();else setPwr(compPID(temp));
        Serial.print(now/1000.0f,1);Serial.print(",");Serial.print(temp,2);Serial.print(",");
        Serial.print(spA,2);Serial.print(",");Serial.print(spT,2);Serial.print(",");
        Serial.print(lPwm);Serial.print(",");Serial.print(Kp,3);Serial.print(",");
        Serial.print(Ki,4);Serial.print(",");Serial.print(Kd,3);Serial.print(",AUTO,");
        Serial.println(isnan(temp2)?0.0f:temp2,2);
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
        Serial.print(frzReady?",FREEZE_READY,":",FREEZE,");
        Serial.println(isnan(temp2)?0.0f:temp2,2);
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
        Serial.print(Kd,3);Serial.print(",MAN,");
        Serial.println(isnan(temp2)?0.0f:temp2,2);
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
