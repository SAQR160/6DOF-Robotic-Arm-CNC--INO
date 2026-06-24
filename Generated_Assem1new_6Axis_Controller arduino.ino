/* Assem1new six-axis controller. Tool output is hard disabled. */
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
constexpr uint8_t N=6; constexpr uint32_t BAUD=115200;
constexpr uint8_t STEP[N]={2, 4, 6, 8, 10, 12}, DIR[N]={3, 5, 7, 9, 11, 13}, HOME[N]={22, 24, 26, 28, 30, 32}, MAXL[N]={23, 25, 27, 29, 31, 33};
constexpr uint8_t ENABLE=36, ESTOP=37, TOOL=44;
constexpr bool ENABLE_LOW=true, HOME_LOW=false, MAX_LOW=false, ESTOP_LOW=false, REQUIRE_HOME=true;
constexpr uint8_t POS_DIR[N]={1, 1, 1, 1, 1, 1}; constexpr int8_t HOME_DIR[N]={-1, -1, -1, -1, -1, -1};
constexpr float HOME_FAST[N]={500.000000f, 350.000000f, 350.000000f, 450.000000f, 450.000000f, 500.000000f}, HOME_SLOW[N]={120.000000f, 80.000000f, 80.000000f, 100.000000f, 100.000000f, 120.000000f}, MAX_RATE[N]={4000.000000f, 1800.000000f, 1800.000000f, 2200.000000f, 2200.000000f, 2600.000000f};
constexpr long BACKOFF[N]={500L, 300L, 300L, 250L, 250L, 300L}, SEARCH_MAX[N]={350000L, 50000L, 50000L, 40000L, 40000L, 60000L}, MIN_STEPS[N]={-128000L, -15111L, -12000L, -7556L, -7556L, -6400L}, MAX_STEPS[N]={128000L, 15111L, 12000L, 7556L, 7556L, 6400L};
constexpr uint16_t PULSE_US=5, DIR_US=10, MIN_TICK_US=20;
constexpr uint32_t MAX_SEGMENT_US=60000000;
long pos[N]={0,0,0,0,0,0}; bool homed[N]={false,false,false,false,false,false}; bool enabled=false;
char linebuf[240]; size_t llen=0;
bool active(uint8_t pin,bool low){bool islow=digitalRead(pin)==LOW;return low?islow:!islow;}
bool estop(){return active(ESTOP,ESTOP_LOW);} bool homeActive(uint8_t a){return active(HOME[a],HOME_LOW);} bool maxActive(uint8_t a){return active(MAXL[a],MAX_LOW);}
void toolOff(){digitalWrite(TOOL,LOW);} void enableDrivers(bool on){enabled=on;digitalWrite(ENABLE,ENABLE_LOW?(on?LOW:HIGH):(on?HIGH:LOW));}
bool allHomed(){for(uint8_t a=0;a<N;a++)if(!homed[a])return false;return true;}
uint8_t dirLevel(uint8_t a,int8_t s){bool p=POS_DIR[a]!=0;return s>0?(p?HIGH:LOW):(p?LOW:HIGH);}
bool blocked(uint8_t a,int8_t s){if(s==HOME_DIR[a]&&homeActive(a))return true;if(s==-HOME_DIR[a]&&maxActive(a))return true;return false;}
bool abortNow(){if(estop())return true;if(Serial.available()>0&&Serial.peek()=='!'){Serial.read();return true;}return false;}
bool oneStep(uint8_t a,int8_t s,float rate){if(abortNow()||blocked(a,s)||rate<=0)return false;digitalWrite(DIR[a],dirLevel(a,s));delayMicroseconds(DIR_US);uint32_t period=(uint32_t)(1000000.0f/rate);digitalWrite(STEP[a],HIGH);delayMicroseconds(PULSE_US);digitalWrite(STEP[a],LOW);pos[a]+=s;if(period>PULSE_US)delayMicroseconds(period-PULSE_US);return true;}
bool moveUntil(uint8_t a,int8_t s,float rate,long limit,bool desired){for(long i=0;i<limit;i++){if(homeActive(a)==desired)return true;if(!oneStep(a,s,rate))return homeActive(a)==desired;}return homeActive(a)==desired;}
bool homeAxis(uint8_t a){homed[a]=false;int8_t toward=HOME_DIR[a],away=-toward;if(homeActive(a)&&!moveUntil(a,away,HOME_SLOW[a],BACKOFF[a]*4,false))return false;if(!moveUntil(a,toward,HOME_FAST[a],SEARCH_MAX[a],true))return false;if(!moveUntil(a,away,HOME_SLOW[a],BACKOFF[a]*4,false))return false;if(!moveUntil(a,toward,HOME_SLOW[a],BACKOFF[a]*8,true))return false;pos[a]=0;homed[a]=true;return true;}
bool homeAll(){toolOff();if(estop())return false;enableDrivers(true);for(int a=N-1;a>=0;a--){Serial.print(F("homing:"));Serial.println(a+1);if(!homeAxis((uint8_t)a)){enableDrivers(false);return false;}}return true;}
bool limitsOK(const long t[N]){for(uint8_t a=0;a<N;a++)if(t[a]<MIN_STEPS[a]||t[a]>MAX_STEPS[a])return false;return true;}
bool ratesOK(const long t[N],uint32_t us){if(us==0||us>MAX_SEGMENT_US)return false;for(uint8_t a=0;a<N;a++){float r=(float)labs(t[a]-pos[a])*1000000.0f/(float)us;if(r>MAX_RATE[a]+0.01f)return false;}return true;}
bool moveSegment(const long target[N],uint32_t us){
  if(REQUIRE_HOME&&!allHomed())return false;
  if(estop()||!limitsOK(target)||!ratesOK(target,us))return false;
  unsigned long delta[N],acc[N]={0,0,0,0,0,0},maxd=0; int8_t sign[N];
  for(uint8_t a=0;a<N;a++){long d=target[a]-pos[a];delta[a]=labs(d);sign[a]=d>=0?1:-1;if(delta[a]>maxd)maxd=delta[a];if(delta[a]&&blocked(a,sign[a]))return false;digitalWrite(DIR[a],dirLevel(a,sign[a]));}
  delayMicroseconds(DIR_US);
  if(maxd==0){uint32_t st=micros();while((uint32_t)(micros()-st)<us)if(abortNow())return false;return true;}
  const uint32_t baseTick=us/maxd,remainder=us%maxd;if(baseTick<MIN_TICK_US)return false;
  uint32_t targetElapsed=0,remainderAccumulator=0;const uint32_t st=micros();
  for(unsigned long tick=0;tick<maxd;tick++){
    targetElapsed+=baseTick;remainderAccumulator+=remainder;if(remainderAccumulator>=maxd){targetElapsed++;remainderAccumulator-=maxd;}
    while((uint32_t)(micros()-st)<targetElapsed)if(abortNow())return false;
    bool pulse[N]={false,false,false,false,false,false};
    for(uint8_t a=0;a<N;a++){acc[a]+=delta[a];if(acc[a]>=maxd){acc[a]-=maxd;if(blocked(a,sign[a]))return false;pulse[a]=true;}}
    for(uint8_t a=0;a<N;a++)if(pulse[a])digitalWrite(STEP[a],HIGH);delayMicroseconds(PULSE_US);
    for(uint8_t a=0;a<N;a++)if(pulse[a]){digitalWrite(STEP[a],LOW);pos[a]+=sign[a];}
  }
  for(uint8_t a=0;a<N;a++)pos[a]=target[a];return true;
}
bool parseLong(char* s,long& v){if(!s)return false;char* e=nullptr;v=strtol(s,&e,10);return e!=s&&*e=='\0';}
bool parseUInt(char* s,uint32_t& v){if(!s)return false;char* e=nullptr;unsigned long x=strtoul(s,&e,10);if(e==s||*e!='\0')return false;v=(uint32_t)x;return true;}
void status(){Serial.print(F("status,homed="));Serial.print(allHomed());Serial.print(F(",estop="));Serial.print(estop());Serial.print(F(",enabled="));Serial.print(enabled);Serial.print(F(",pos="));for(uint8_t a=0;a<N;a++){if(a)Serial.print(',');Serial.print(pos[a]);}Serial.println();}
void position(char* s){char* tok=strtok(s,",");long seq,tool;uint32_t us;long target[N];if(!tok||strcmp(tok,"P")||!parseLong(strtok(nullptr,","),seq)||!parseUInt(strtok(nullptr,","),us)){Serial.println(F("error,bad-header"));return;}for(uint8_t a=0;a<N;a++)if(!parseLong(strtok(nullptr,","),target[a])){Serial.println(F("error,bad-target"));return;}if(!parseLong(strtok(nullptr,","),tool)||strtok(nullptr,",")!=nullptr){Serial.println(F("error,bad-fields"));return;}toolOff();enableDrivers(true);if(!moveSegment(target,us)){enableDrivers(false);Serial.println(estop()?F("error,estop"):F("error,motion-rejected"));return;}Serial.print(F("ok,"));Serial.println(seq);}
void process(char* s){if(!strcmp(s,"PING")){Serial.println(F("ready"));return;}if(!strcmp(s,"STATUS")){status();return;}if(!strcmp(s,"HOME")){Serial.println(homeAll()?F("ok,home"):F("error,home"));return;}if(!strcmp(s,"DISABLE")){toolOff();enableDrivers(false);for(uint8_t a=0;a<N;a++)homed[a]=false;Serial.println(F("ok,disabled"));return;}if(s[0]=='P'&&s[1]==','){position(s);return;}Serial.println(F("error,unknown"));}
void setup(){Serial.begin(BAUD);for(uint8_t a=0;a<N;a++){pinMode(STEP[a],OUTPUT);pinMode(DIR[a],OUTPUT);pinMode(HOME[a],INPUT_PULLUP);pinMode(MAXL[a],INPUT_PULLUP);digitalWrite(STEP[a],LOW);}pinMode(ENABLE,OUTPUT);pinMode(ESTOP,INPUT_PULLUP);pinMode(TOOL,OUTPUT);toolOff();enableDrivers(false);delay(300);Serial.println(F("ready"));}
void loop(){if(estop()){toolOff();enableDrivers(false);for(uint8_t a=0;a<N;a++)homed[a]=false;}while(Serial.available()){char c=(char)Serial.read();if(c=='!'){toolOff();enableDrivers(false);llen=0;Serial.println(F("error,aborted"));continue;}if(c=='\n'){linebuf[llen]='\0';if(llen)process(linebuf);llen=0;}else if(c!='\r'){if(llen<sizeof(linebuf)-1)linebuf[llen++]=c;else{llen=0;Serial.println(F("error,line-too-long"));}}}}
