/* New MeteoV2 */
// 14/08/2021
// ESP32, DHT22, DS3232, BMP280

// I2C add :DS3232:0X68, EEPROM:0X57, BMP280:0X76

// utilisation du copro ULP pour le comptage des impulsions Rain
// le processeur principal est mis en deep sleep la plupart du temps
// il est reveillé toute les 1mn par le RTC externe, la durée de sleep de ULP doit etre superieur 
// elle est fixée à 115s, toutes les mn reveil recupere le comptage et retour sleep
// la valeur "interval" fixe la periodicité de upload des data sur serveur
// un abonnement MQTT permet de recevoir des commandes depuis le serveur
// mode test cavalier sur PinTest
// SSID/PWD sous forme de liste 5 elements, le 5eme element(4) peut etre modifié par mqtt message

// to do
// sur nouveau chargement new magic, coeff calibration sont perdu?
// 

// temps de fonctionnement sauf premier lancement
// toutes les 1 mn durée 200ms environ
// toutes les 15 mn durée 4s, 6s premiere fois

// installation https://github.com/duff2013/ulptool
// si pb compilation voir https://github.com/duff2013/ulptool/issues/78
// ajouter dans platforms.local.txt : 
// recipe.hooks.core.prebuild.01.pattern.windows=cmd /c if exist "{build.source.path}\*.s" copy /y "{build.source.path}\*.s" "{build.path}\sketch\"

// 16/04/2023
// Arduino IDE 1.8.19 ESP32 1.0.2 (garder cette version pour compatibilité SPIFFS)
// 973662 octets (74%), 41376 octets (12%)

const int ver = 10;
int Magique   = 3;

#include "Arduino.h"
#include <Wire.h>
#include "driver/gpio.h"   // ULP
#include "driver/rtc_io.h" // ULP
#include "esp32/ulp.h"     // ULP
#include "ulp_main.h"      // include ulp header you will create
#include "ulptool.h"       // Custom binary loader
#include <DS3232RTC.h>     // Horloge http://github.com/JChristensen/DS3232RTC
#include "DHT.h"           // Capteur T°C et Humidity
#include <PubSubClient.h>  // Publication MQTT
#include <SPIFFS.h>
#include <EEPROM.h>        // variable en EEPROM(SPIFFS)
#include <ArduinoOTA.h>
#include <ESP32httpUpdate.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <FS.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <credentials_new_guico.h>

#define PinRain GPIO_NUM_13 // pullup hard 22k
#define DHTPIN          4   // Digital pin connected to the DHT sensor
#define PinBattProc     35
#define PinAlarmRTC     32
#define PinPannSol      39
#define PinTest         27  // pin test ouvert = test, = 0 normal
#define PinSDA          17  // 21 Pin par defaut
#define PinSCL          16  // 22 Pin par defaut, ne fonctionne pas
#define DHTTYPE DHT22       // DHT 22  (AM2302), AM2321

const String soft = "ESP32_Meteo.ino.d32."; 	// nom du soft

// Unlike the esp-idf always use these binary blob names
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

uint32_t TIME_TO_SLEEP = 115; // doit etre > 60 et < 120 evite reveil avant Alarme externe
// uint32_t TempoULP = 10; // en s
uint32_t uS_TO_S_FACTOR = 1000000L; // conversion s en µs
float TempC = 0;
RTC_DATA_ATTR float maxTempC;             // température max sur 24H
RTC_DATA_ATTR float minTempC;             // température min sur 24H
float Humid = 0;
float pressure = 1000;
String adressemac;
String adresseip;
String ssidconnected;							// apres connexion wifi contient le ssid+nbr de tentative de cnx
String Sbidon="";
char filecalibration[11] = "/coeff.txt";    // fichier en SPIFFS contenant les data de calibration
char filesauvdata[11]    = "/sauvdata_";    // fichiers (1 par sauvegarde, max 100) en SPIFFS contenant les data sauvegarde defaut mqtt
char fileSSID[10]        = "/ssid.txt";     // fichiers contenant lites SSID et pwd
int CoeffTension[2];          // Coeff calibration Tension
int CoeffTensionDefaut = 7000;// Coefficient par defaut
long Vbatt = 0;
long Vpansol = 0;
bool SPIFFS_present = false;
bool FlagTest = false;
byte confign = 0;                   // position enregistrement config EEPROM
// bool fileStatus[100] = {0}; // etat file retransmit true si dispo
struct  config_t           // Structure configuration sauvée en EEPROM
{
  int     magic;           // num magique
  char    mqttServer[26];
  char    mqttUserName[11];// MQTT User
  char    mqttPass[16];    // MQTT pass
  char    sendTopic[17];
  char    receiveTopic[17];
  char    permanentTopic[20];
  int     mqttPort;
  float   rain_to_mm;       // conversion beat->mm
  int     interval;         // interval upload en s
} ;
config_t config;

// varialbles en RTC
RTC_DATA_ATTR int rainHour[60] = {0}; // rain per minute number of beat
RTC_DATA_ATTR int rainlasthour = 0;   // rain last hour nbr of beat
RTC_DATA_ATTR int dailyrain = 0;      // rain over 24h nbr of beat
RTC_DATA_ATTR int lasthour = 0;       // memorise derniere heure pour calcul dailyrain
RTC_DATA_ATTR bool Flagreadfirst = true; // premiere lecture datameteo
RTC_DATA_ATTR int last_ssid = 99;     // memorise indice dernier SSID utilisé, 99 au départ
RTC_DATA_ATTR int file2retransmit = 0; // nombre file stockée pour retransmission

DHT dht(DHTPIN, DHTTYPE);
DS3232RTC RTC;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
//https://stackoverflow.com/questions/58719387/writing-and-reading-object-into-esp32-flash-memory-arduino

//---------------------------------------------------------------------------
void setup(){
  Serial.begin(115200);
  Serial.println("");

  pinMode(PinAlarmRTC , INPUT_PULLUP);
  pinMode(PinTest     , INPUT_PULLUP);
  adcAttachPin(PinBattProc);
  adcAttachPin(PinPannSol);

  if (digitalRead(PinTest) == 0) { // lire strap test, si = 0 test
    FlagTest = true;
    printf("mode test\n");
  }

  EEPROM.begin(512);
  EEPROM.get(confign, config); // lecture config
  if (config.magic != Magique) {
    /* verification numero magique si different
       erreur lecture EEPROM ou carte vierge
    	 on charge les valeurs par défaut
    */
    printf("Nouvelle Configuration !\n");
    config.magic      = Magique;
    config.rain_to_mm = 0.2794;
    config.interval   = 900 ; //secondes
    // declaration des Topics MQTT
    strcpy(config.mqttServer,tempServer);
    strcpy(config.sendTopic,temptopic);    
    strcat(config.sendTopic,"/input");
    strcpy(config.receiveTopic,temptopic);
    strcat(config.receiveTopic,"/output");
    strcpy(config.permanentTopic,temptopic);
    strcat(config.permanentTopic,"/permanent");
    config.mqttPort = tempmqttPort;    
    strcpy(config.mqttUserName,tempmqttUserName);
    strcpy(config.mqttPass,tempmqttPass);
  
    EEPROM.put(confign, config);
    EEPROM.commit();
    delay(100);
  }
  EEPROM.end();
  if(FlagTest)PrintEEPROM();
  
  if (!SPIFFS.begin(true)) {
    printf("SPIFFS initialisation failed...\n");
    SPIFFS_present = false;
  } else {
    printf("SPIFFS initialised... file access enabled...\n");
    SPIFFS_present = true;
  }
  Wire.begin(PinSDA,PinSCL);

  OuvrirFichierSSID();

  OuvrirFichierCalibration();
  dht.begin();
  Parametrage_RTC();
  DecisionWakeup();

  mqttClient.setBufferSize(512);
  mqttClient.setSocketTimeout(5);
}
//---------------------------------------------------------------------------
void loop(){
  static byte lastsecond = 0;
  static byte lastminute = 0;
  
  if(FlagTest){
    static bool first = true;
    if(first){
      Read_data_meteo();
      manage_upload_data(false);
      first = false;
    }
    if(second()!=lastsecond && second()%10 == 0){
      lastsecond = second();
      printf("%02d/%02d/%d %02d:%02d:%02d\n",day(),month(),year(),hour(),minute(),second());
    }
  }else{
    Read_data_meteo();
    printf("%02d/%02d/%d %02d:%02d:%02d\n",day(),month(),year(),hour(),minute(),second());
  }
    
  if (lasthour != hour()){ // Heure
    lasthour = hour();
    calcDailyrain();
    if(hour() == 0 && minute() == 0){// 00H00 fin de journée
      Read_data_meteo();
      OuvrirFichierCalibration(); // patch relecture des coeff perdu
      mesureTension();
      manage_fin_journee();
    }
    if(hour() == 1 && minute() == 0){// 01H00 verifier si updatesoft dispo
      MajSoft();
    }
  }
  if(FlagTest){
    if(lastminute != minute() && minute()%1 == 0){// %15
      lastminute = minute();
      Read_data_meteo();
      OuvrirFichierCalibration(); // patch relecture des coeff perdu
      mesureTension();
      // mesure batterie et PanSol, connexion Wifi, envoie data vers serveur, lecture Abonnement MQTT
      manage_upload_data(false);
      
    }
    mqttClient.loop();
  }else{
    if(minute()%(config.interval/60) == 0){// %15
      lastminute = minute();
      Read_data_meteo();
      // printf("interval : %d\n",config.interval);
      OuvrirFichierCalibration(); // patch relecture des coeff perdu
      mesureTension();
      // mesure batterie et PanSol, connexion Wifi, envoie data vers serveur, lecture Abonnement MQTT
      manage_upload_data(false);
    }

    delay(100); // Obligatoire
    DebutSleep();
  }

  delay(1);
}
//---------------------------------------------------------------------------
void manage_fin_journee(){
  manage_upload_data(true);
  dailyrain = 0;
  minTempC = TempC;
  maxTempC = TempC;
}
//---------------------------------------------------------------------------
void manage_upload_data(bool Finjournee){
  // Finjournee = true -> mettre heure 23:59:59 pour le calcul finjournée du serveur
  mqttClient.setServer(config.mqttServer, config.mqttPort);
  mqttClient.setCallback(callback);
  mqttClient.setBufferSize(300);// long message "param"
  printf("nbr file retransmit:%d\n",file2retransmit);
  if (connectWIFI()) {	// connexion Wifi, si pas de Wifi dispo retour deepsleep
    mqttClient.loop();
    if (!mqttClient.connected() && WiFi.status() == WL_CONNECTED) {
      reconnect();
    }
    bool rep = false;
    Sbidon = prepare_data(Finjournee);
    rep = send_mqtt_data(0,Sbidon);
    // byte nfile = 0; // numero file à lire/envoyer
    if(!rep){// echec mqtt, sauvegarde data pour envoi ulterieur
      saveFileData();
    } else { // si ok, s'il y a des fichiers enregistrer, on les envoie
      do {
        if(file2retransmit > 0){// tant qu'il y a des fichiers
          // do{
          //   nfile ++;
          // }while(fileStatus[nfile] != true); // cherche premier fichier disponible
          Sbidon = ReadFichierSauvData(file2retransmit); // lire fichier
          Serial.print("read file retransmit:"),Serial.println(file2retransmit);
          rep = send_mqtt_data(0,Sbidon); // envoyer fichier
          if(rep){
            SPIFFS.remove(filesauvdata+String(file2retransmit));
            // fileStatus[nfile] = false;
            // nfile ++;
            file2retransmit --; // si mqtt OK, on decremente
          }
          } else {
            rep = false;// plus de fichier
          }        
      }while(rep);
    }

    // attente abonnement
    long debut = millis();
    printf("Boucle attente mqtt callback\n");
    while (millis() - debut < 1000 ){
      mqttClient.loop();
    }
  } else {
    saveFileData();
  }
}
//---------------------------------------------------------------------------
// sauvegarde data pour envoie ulterieur
void saveFileData(){
  if(file2retransmit < 99){
    file2retransmit ++;
    RecordFichierSauvData(Sbidon,file2retransmit);
    // fileStatus[file2retransmit] = true;
    Serial.print("save file retransmit:"),Serial.println(file2retransmit);
  }
}
//---------------------------------------------------------------------------
// Send mqtt data
// ntopic = 0 for sendTopic, = 1 for permanentTopic
bool send_mqtt_data(byte ntopic,String data){  
  Serial.print("len data MQTT: "),Serial.print(data.length()),Serial.print(" ; "),Serial.print(data),Serial.print(" ; topic : "),Serial.println(ntopic);
  char replybuffer[data.length()+1];
  data.toCharArray(replybuffer, data.length()+1);
  int n = 0;
  switch (ntopic){
    case 0:
      n = mqttClient.publish(config.sendTopic, replybuffer);
      break;
    case 1:
      n = mqttClient.publish(config.permanentTopic,replybuffer,true);
      break;
  }
//***************test erreur mqtt *****************
  // static byte cpt = 0;
  // static bool flagfin = false;
  // if(!flagfin){
  //   if( cpt > 1 && cpt < 10){
  //     n = 0;
  //     cpt ++;
  //   } else if(cpt >= 10){
  //     cpt = 0;
  //     flagfin=true;
  //   }else { cpt ++;}    
  // }
//***************test erreur mqtt *****************
  if (n > 0) {
    Serial.print("send mqtt OK:"),Serial.println(n);
    // printf("send mqtt OK: %d\n",n);
    return true;
  } else {
    Serial.print("send mqtt KO:"),Serial.println(n);
    // printf("send mqtt KO: %d\n",n);
    return false;
  }  
}
//---------------------------------------------------------------------------
String prepare_data(bool Finjournee){
  // Finjournee = true -> mettre heure 23:59:59 pour le calcul finjournée du serveur
  // json contenant data à envoyer
  // {"meteo":{"temp":TempC,"humid":Humid,"rain1":rainlasthour,"rain24h":dailyrain,"vbatt":vbatt,
  // "vpansol":vpansol,"rssi":rssi,"versoft":ver,"SSID":ssidconnected}}
  // {"meteo":{"temp":25.02,"humid":85,"rain1":25.5,"rain24h":50.9,"vbatt":4200,"vpansol":5600,"rssi":-85,"versoft":"000","SSID":"Nordnet_FB00-5"}}
  DynamicJsonDocument doc(384);  //https://arduinojson.org/v6/assistant/
  JsonObject meteo = doc.createNestedObject("meteo");
  char bidon[20];
  if(Finjournee){
    time_t t = now();
    t -= 86400; // remonte à hier
    sprintf(bidon,"%d-%02d-%02d %02d:%02d:%02d",year(t),month(t),day(t),23,59,59);
  } else {
    sprintf(bidon,"%d-%02d-%02d %02d:%02d:%02d",year(),month(),day(),hour(),minute(),second());
  }
  meteo["time"] = bidon;
  sprintf(bidon,"%.2lf",TempC);// meteo["temp"] = TempC;
  meteo["temp"] = bidon;
  sprintf(bidon,"%.2lf",maxTempC);// meteo["maxtemp"] = maxTempC;
  meteo["maxtemp"] = bidon;
  sprintf(bidon,"%.2lf",minTempC);// meteo["mintemp"] = minTempC;
  meteo["mintemp"] = bidon;
  sprintf(bidon,"%.2lf",minTempC);// meteo["humid"] = Humid;
  meteo["humid"] = bidon;
  meteo["pression"] = pressure;
  sprintf(bidon,"%.2lf",rainlasthour* config.rain_to_mm); //meteo["rain1h"] = rainlasthour* config.rain_to_mm;
  meteo["rain1h"] = bidon;
  sprintf(bidon,"%.2lf",dailyrain* config.rain_to_mm);//meteo["rain24h"] = dailyrain* config.rain_to_mm;
  meteo["rain24h"] = bidon;
  meteo["vbatt"] = Vbatt;
  meteo["vpansol"] = Vpansol;
  meteo["interdata"] = config.interval;
  meteo["rssi"] = String(WiFi.RSSI());
  meteo["versoft"] = ver;
  meteo["SSID"] = ssidconnected; // "Nordnet_FB00-5"
  Sbidon = "";
  serializeJson(doc, Sbidon);
  // Serial.print("data MQTT:"),Serial.println(Sbidon);
  return Sbidon;
}
//---------------------------------------------------------------------------
void reconnect() {
  // Loop until we're reconnected
  byte cpt = 0;
  while (!mqttClient.connected()) { // mqttClient.setSocketTimeout(5)
    printf("Attempting MQTT connection...\n");
    // Attempt to connect
    if (mqttClient.connect("mymeteo", config.mqttUserName, config.mqttPass)) {
      printf("connected\n");
      // Subscribe
      mqttClient.subscribe(config.receiveTopic);  //("meteo/output");
      mqttClient.subscribe(config.permanentTopic);//("meteo/permanent");
    } else {
      printf("failed, rc= %d try again in 5 seconds\n",mqttClient.state());
      // Wait 5 seconds before retrying
      delay(1);
    } 
    if(cpt ++ > 0) return;
  }
}
//---------------------------------------------------------------------------
void DecisionWakeup(){
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  // printf("wake up :%d\n",cause);
  if ((cause != ESP_SLEEP_WAKEUP_ULP) && (cause != ESP_SLEEP_WAKEUP_EXT1))  {
      printf("Not ULP wakeup, initializing ULP\n");
      init_ulp_program();
  } else {
      printf("ULP wakeup or EXT1, saving pulse count\n");
      update_pulse_count();
  }
  // Serial.print("wake up timer:"),Serial.println(ESP_SLEEP_WAKEUP_TIMER);
  // Serial.print("wake up ULP:"),Serial.println(ESP_SLEEP_WAKEUP_ULP);
  // Serial.print("wake up undefined:"),Serial.println(ESP_SLEEP_WAKEUP_UNDEFINED);
  // Serial.print("wake up EXT:"),Serial.println(ESP_SLEEP_WAKEUP_EXT1);
  // Serial.print("wake up :"),Serial.println(cause);
}
//---------------------------------------------------------------------------
void DebutSleep() {
  // preparation Deep Sleep wake up EXT1(Alarm RTC)
  const uint64_t ext_wakeup_pin_1_mask = 1ULL << PinAlarmRTC;
  esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ALL_LOW);
  ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR));
  // preparation Deep Sleep wake up ULP 
  ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
  printf("Entering deep sleep \n\n");
  esp_deep_sleep_start();
  delay(100);
  // le programme n'ira pas plus moin
}
//---------------------------------------------------------------------------
void Parametrage_RTC(){
  setSyncProvider(RTC.get);   // the function to get the time from the RTC

	if(timeStatus() != timeSet){
		printf("Unable to sync with the RTC\n");
	}
	else{
		printf("Heure système mise à jour par RTC\n"); 
	}
  // initialize the alarms 
	RTC.setAlarm(ALM1_MATCH_SECONDS, 0, 0, 0, 0);	// set Alarm 1 to occur at 0 seconds after every minute
	RTC.alarm(ALARM_1); 													// clear the alarm flag	
	RTC.squareWave(SQWAVE_NONE); 									// configure the INT/SQW pin for "interrupt" operation (disable square wave output)	
	RTC.alarmInterrupt(ALARM_1, true); 					  // enable interrupt output for Alarm 1

	// setTime(17, 10, 00, 4, 8, 2021);   //set the system time to 23h31m30s on 13Feb2009
	// RTC.set(now());                  //set the RTC from the system time
  // setSyncProvider(RTC.get);   // the function to get the time from the RTC
}
//---------------------------------------------------------------------------
void Update_rain_count(uint32_t beat){
  rainHour[minute()] = beat;

  for (int m=0;m<60;m++){
    printf("%d;",rainHour[m]);
  }
  printf("\n");

  calcRain60mn();
}
//---------------------------------------------------------------------------
void calcDailyrain(){
  for (int i = 0 ; i < 60 ; i++) {
    dailyrain += rainHour[i];
  }
  printf("Rain24h: %d\n",dailyrain);
}
//---------------------------------------------------------------------------
void calcRain60mn() {
  //Calculate amount of rainfall for the last 60 minutes
  rainlasthour = 0;
  for (int i = 0 ; i < 60 ; i++) {
    rainlasthour += rainHour[i];
  }
  
  printf("rainlasthour: %d\n",rainlasthour);
}
//---------------------------------------------------------------------------
void Read_data_meteo(){
  
  Humid = dht.readHumidity();
  TempC = dht.readTemperature();
  if(Flagreadfirst){
    minTempC = TempC;
    maxTempC = TempC;
    Flagreadfirst = false;
  }
  if(TempC < minTempC) minTempC = TempC;
  if(TempC > maxTempC) maxTempC = TempC;
  if (isnan(Humid) || isnan(TempC)) {
    printf("Failed to read from DHT sensor!\n");
    return;
  }
  float hic = dht.computeHeatIndex(TempC, Humid, false);

  printf("Humidity: %3.2f % Temperature: %2.2f°C minTemp: %2.2f°C maxTemp: %2.2f°C Heat index: %2.2f°C\n",Humid,TempC,minTempC,maxTempC,hic);
  // printf("Temp capteur pression = %2.2f°C; ",tempc_capteurpression);
  // printf("Pressure = %f",pressure);
  // printf("; Altitude = %d\n",altitude);
}
//---------------------------------------------------------------------------
static void init_ulp_program(void) {
  esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
          (ulp_main_bin_end - ulp_main_bin_start)/ sizeof(uint32_t));
  ESP_ERROR_CHECK(err);

  /* GPIO used for pulse counting. */
  gpio_num_t gpio_num = PinRain; // origine GPIO_NUM_0
  assert(rtc_gpio_desc[gpio_num].reg && "GPIO used for pulse counting must be an RTC IO");

  /* Initialize some variables used by ULP program.
    * Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
    * These variables are declared in an auto generated header file,
    * 'ulp_main.h', name of this file is defined in component.mk as ULP_APP_NAME.
    * These variables are located in RTC_SLOW_MEM and can be accessed both by the
    * ULP and the main CPUs.
    *
    * Note that the ULP reads only the lower 16 bits of these variables.
    */
  ulp_debounce_counter = 3;
  ulp_debounce_max_count = 3;
  ulp_next_edge = 0;
  ulp_io_number = rtc_gpio_desc[gpio_num].rtc_num; /* map from GPIO# to RTC_IO# */
  ulp_edge_count_to_wake_up = 10;

  /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
  rtc_gpio_init(gpio_num);
  rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_dis(gpio_num);
  rtc_gpio_pullup_dis(gpio_num);
  rtc_gpio_hold_en(gpio_num);

  /* Disconnect GPIO12 and GPIO15 to remove current drain through
    * pullup/pulldown resistors.
    * GPIO12 may be pulled high to select flash voltage.
    */
  rtc_gpio_isolate(GPIO_NUM_12);
  rtc_gpio_isolate(GPIO_NUM_15);
  esp_deep_sleep_disable_rom_logging(); // suppress boot messages

  /* Set ULP wake up period to T = 20ms.
    * Minimum pulse width has to be T * (ulp_debounce_counter + 1) = 80ms.
    */
  ulp_set_wakeup_period(0, 20000);

  /* Start the program */
  err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
  ESP_ERROR_CHECK(err);
}
//---------------------------------------------------------------------------
static void update_pulse_count(void) {
  /* ULP program counts signal edges, convert that to the number of pulses */
  uint32_t pulse_count_from_ulp = (ulp_edge_count & UINT16_MAX) / 2;
  /* In case of an odd number of edges, keep one until next time */
  ulp_edge_count = ulp_edge_count % 2;
  
  printf("Pulse count from ULP: %5d\n", pulse_count_from_ulp);
  Update_rain_count(pulse_count_from_ulp);
}
//---------------------------------------------------------------------------
bool connectWIFI() {
  // Connexion à un réseau WIFI
  byte retries = 0;
  printf("lastssid:%d\n",last_ssid);
  if(last_ssid !=99){// si last_ssid valide(pas au démarrage)
    // cnx au dernier ssid    
    WiFi.begin(ssid[last_ssid], pwd[last_ssid]);
    printf("connexion à %s\n",ssid[last_ssid]);
    while (WiFi.status() != WL_CONNECTED) {
      delay(200);
      printf(".");
      retries++;
      if (retries > 15) {
        retries = 0;
        break;
      }
    }
  }
  if(WiFi.status() != WL_CONNECTED){
    printf("scan start\n");
    int ssidnbr = 0;
    int ssiddispo[nssid];
    int n = WiFi.scanNetworks();
    printf("scan done : ");
    if (n == 0) {
      printf("no networks found\n");
      return false;
    } else {
      printf("%d",n);
      printf(" networks found\n");
      for (int i = 0; i < n; ++i) {
        // Print SSID and RSSI for each network found
        Serial.print(i);
        Serial.print(": ");
        Serial.print(WiFi.SSID(i));
        Serial.print(" (");
        Serial.print(WiFi.RSSI(i));
        Serial.println(")");
        delay(10);
      }
    }
    for (byte i = 0; i < nssid; i ++) {	// cherche reseau connu parmi present
      for (byte j = 0; j < n; j ++) {
        int rc = strcmp(WiFi.SSID(j).c_str(), ssid[i]);
        if (rc == 0) {
          ssiddispo[ssidnbr] = i;	// liste reseau connu
          ssidnbr ++;							// Qte reseau connu
          printf("ssidnbr = %d, ",ssidnbr);
        }
      }
    }
    if(ssidnbr == 0)return false;// pas de ssid connu
    
    for (int i = 0; i < ssidnbr; i ++) {
      printf("SSID dispo  %s\n",ssid[ssiddispo[i]]);
    }
    retries = 0;
    byte i = 0;
    do {
      WiFi.begin(ssid[ssiddispo[i]], pwd[ssiddispo[i]]);
      printf("connexion à %s\n",ssid[ssiddispo[i]]);
      last_ssid = ssiddispo[i];// memorisation du lastssid
      while (WiFi.status() != WL_CONNECTED) {
        delay(200);
        printf(".");
        retries++;
        if (retries > 25) {
          retries = 0;
          //break;
          if (i < ssidnbr) {
            i++;
            printf("%d\n",i);
            break;
          }
          else {
            printf("tous les reseaux balayés sans connexion reussie\n");
            last_ssid = 99;// pas de lastssid
            return false;	// tous les reseau balayés sans connexion reussie
          }
        }
      }
    } while (WiFi.status() != WL_CONNECTED);
  }
  
  adressemac     = WiFi.macAddress();
  adresseip      = WiFi.localIP().toString();
  ssidconnected  = String(WiFi.SSID());
  ssidconnected += "-";
  ssidconnected += String(retries);
  
  Serial.print("WiFi connected;"),Serial.print(ssidconnected),Serial.print(", RSSI:"),Serial.println(WiFi.RSSI());
  Serial.print("IP  address: "),Serial.println(adresseip);
  // Serial.print("MAC address: "),Serial.println(adressemac);
  return true;
}
//---------------------------------------------------------------------------
void callback(char* topic, byte* buffer, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". length: ");
  Serial.print(length);
  Serial.print(". Message: ");
  Sbidon = "";  
  for (int i = 0; i < length; i++) {
    Serial.print((char)buffer[i]);
    Sbidon += (char)buffer[i];
  }
  Serial.println("");
  if(length == 0) return; // sortir sans rien faire

  // Serial.print("topic:"),Serial.print(topic);
  // Serial.print(", receivetopic:"),Serial.println(config.receiveTopic);
  if (String(topic) == config.receiveTopic) { // nouveau message arrivé
    Serial.print("receive topic:"),Serial.println(Sbidon);
    if(Sbidon.indexOf("MajHeure;") == 0){
      //format : MajHeure;2021-9-13 12:15:17
      byte pv = Sbidon.indexOf(";");
      byte t1 = Sbidon.indexOf("-");
      byte t2 = Sbidon.indexOf("-",t1+1);
      byte e  = Sbidon.indexOf(" ",t2+1);
      byte p2_1 = Sbidon.indexOf(":",e+1);
      byte p2_2 = Sbidon.indexOf(":",p2_1+1);

      int Y = (Sbidon.substring(pv+1,t1)).toInt();//Y
      byte M = (Sbidon.substring(t1+1,t2)).toInt();//M
      byte D = (Sbidon.substring(t2+1,e)).toInt();//D
      byte H = (Sbidon.substring(e+1,p2_1)).toInt();//H
      byte m = (Sbidon.substring(p2_1+1,p2_2)).toInt();//m
      byte s = (Sbidon.substring(p2_2+1,Sbidon.length())).toInt();//s
      setTime(H, m, s, D, M, Y);
      // setTime(17, 10, 00, 4, 8, 2021);   //set the system time to 23h31m30s on 13Feb2009
	    RTC.set(now());                  //set the RTC from the system time
      printf("Heure mise à jour; ");
      if(s<58)s+=1;
      printf("%02d/%02d/%d %02d:%02d:%02d\n",D,M,Y,H,m,s);
    }else if(Sbidon.indexOf("MajSoft") == 0){
      MajSoft();
    }else if(Sbidon.indexOf("interval=") == 0){
      if(Sbidon.substring(9).toInt() > 0){
        config.interval = Sbidon.substring(9).toInt();
        Serial.print("interval:"),Serial.println(config.interval);
        sauvConfig();
      }
    }else if(Sbidon.indexOf("adc=") == 0){
      // format ; adc=7000;7000
      byte pv2 = Sbidon.indexOf(";",4);
      int adc1 = (Sbidon.substring(4,pv2)).toInt();
      int adc2 = (Sbidon.substring(pv2+1,Sbidon.length())).toInt();
      if(adc1 > 0 && adc2 > 0){
        CoeffTension[0] = adc1;
        CoeffTension[1] = adc2;
        Recordcalib();
        printf("Record Coeff T Batterie = %d, Coeff T PanSol = %d\n",CoeffTension[0],CoeffTension[1]);
      }
    }
  }
  if (String(topic) == config.permanentTopic) { // nouveau message arrivé
    Serial.print("permanent topic:"),Serial.println(Sbidon);
    if(Sbidon == "MajSoft" ){ // verification si nouveau soft dispo et maj
      resetPermanentTopic();
      MajSoft();
    } else if (Sbidon.indexOf("interval=") == 0){
      if(Sbidon.substring(9).toInt() > 0){
        config.interval = Sbidon.substring(9).toInt();
        Serial.print("interval:"),Serial.println(config.interval);
        sauvConfig();
        resetPermanentTopic();
      }
    }else if(Sbidon.indexOf("adc=") == 0){ // format ; adc=7000;7000
      byte pv2 = Sbidon.indexOf(";",4);
      int adc1 = (Sbidon.substring(4,pv2)).toInt();
      int adc2 = (Sbidon.substring(pv2+1,Sbidon.length())).toInt();
      if(adc1 > 0 && adc2 > 0){
        CoeffTension[0] = adc1;
        CoeffTension[1] = adc2;
        Recordcalib();
        printf("Record Coeff T Batterie = %d, Coeff T PanSol = %d\n",CoeffTension[0],CoeffTension[1]);
        resetPermanentTopic();
      }
    }else if(Sbidon.indexOf("rst") == 0){ // demande Reset
      resetPermanentTopic();
      printf("Reboot soft\n");
      ESP.restart();
    }else if(Sbidon.indexOf("Dmajheure") == 0){ // demande Mise à l'heure
      resetPermanentTopic();
      send_mqtt_data(0,"Dmajheure"); // envoyer au serveur qui repondra par MajHeure   
    }else if(Sbidon.indexOf("SSID") == 0){ // demande ou modification SSID
      // SSID=myssid,mypassword
      if(Sbidon.indexOf(char(61)) == 4){
        byte posv = Sbidon.indexOf(",",5);
        String newssid = Sbidon.substring(5,posv);
        String newpass = Sbidon.substring(posv+1,Sbidon.length());
        newssid.toCharArray(ssid[4],newssid.length()+1);
        newpass.toCharArray(pwd[4],newpass.length()+1);
      }
      Serial.print(print_ssid_list());
      send_mqtt_data(0,print_ssid_list()); // envoyer liste ssid
      resetPermanentTopic();
    }else if(Sbidon.indexOf("ERASEFILE") == 0){ // demande effacer fichier ou liste file
      if(Sbidon.indexOf(char(61)) == 9){
        String file = Sbidon.substring(10);
        SPIFFS.remove(file);
      }
      listDir(SPIFFS, "/", 0);// liste Dir dans Sbidon
      send_mqtt_data(0,Sbidon.c_str()); // envoyer liste fichier
      resetPermanentTopic();

    }else if(Sbidon.indexOf("READFILE") == 0){ // demande lecture fichier      
      String filename = Sbidon.substring(9);
      Serial.print("file to read:"),Serial.println(filename);
      Sbidon="";
      File f = SPIFFS.open(filename,"r");
      while(f.available()){
        int inchar = f.read();
        Sbidon += char(inchar);
      }
      f.close();
      send_mqtt_data(0,Sbidon.c_str()); // envoyer contenu du fichier
      resetPermanentTopic();
    }
  }
}
//---------------------------------------------------------------------------
//renvoyer "" sur permanentTopic pour eviter repetition
void resetPermanentTopic(){
  send_mqtt_data(1,"");
}
//---------------------------------------------------------------------------
//creation liste SSID/PWD
String print_ssid_list(){
  String reponse = "\n";
  for(int i=0;i<nssid;i++){
    reponse += "ssid/pwd:";
    reponse += ssid[i];
    reponse += "|";
    reponse += pwd[i];
    reponse += "\n";
  }
  return reponse;
}
//---------------------------------------------------------------------------
String ReadFichierSauvData(byte n){
  // n = numero du fichier "\sauvdata_1"
  // fichier comprenant sur une ligne les data non envoyés suite pb mqtt
  String rep = "";
  if (SPIFFS.exists(filesauvdata+String(n))) {
    File f = SPIFFS.open(filesauvdata+String(n), "r");
    rep = f.readStringUntil('\n');
    // Serial.print("lecture sauv:"),Serial.println(rep);
    f.close();
  } else {
    // printf("Creating sauvdata File:%s%s\n",filesauvdata,String(n));
    // RecordFichierSauvData("",n);
  }
  return rep;
}
//---------------------------------------------------------------------------
void RecordFichierSauvData(String data, byte n){
  // creation fichier retransmit n
  File f = SPIFFS.open(filesauvdata+String(n), "w+");
  // Serial.print("sauvdata:"),Serial.println(filesauvdata+String(n));
  // Serial.print("data:"),Serial.println(data);
  f.println(data);
  f.close();
}
//---------------------------------------------------------------------------
// Ouvre le fichier des SSID/pwd
void OuvrirFichierSSID(){
  // premiere fois fichier inexistant, copie char ssid et pwd dedans
  if (SPIFFS.exists(fileSSID)){
    File f = SPIFFS.open(fileSSID, "r");
    for(int i=0;i<nssid;i++){
      Sbidon = f.readStringUntil('\n');
      // Serial.println(Sbidon);
      Sbidon.toCharArray(ssid[i],Sbidon.length()+1);
    }
    for(int i=0;i<nssid;i++){
      Sbidon = f.readStringUntil('\n');
      // Serial.println(Sbidon);
      Sbidon.toCharArray(pwd[i],Sbidon.length()+1);
    }
    f.close();
  } else {
    Serial.print(F("Creating Data File:")), Serial.println(fileSSID); // valeur par defaut
    File f = SPIFFS.open(fileSSID, "w+");
    for(int j=0;j<nssid;j++){
      f.print(ssid[j]);
      f.print('\n');
      // Serial.println(ssid[j]);
    }
    for(int i=0;i<nssid;i++){
      f.print(pwd[i]);
      f.print('\n');
      // Serial.println(pwd[i]);
    }
    f.close();
  }

  // Serial.print(print_ssid_list());
  // SPIFFS.remove(fileSSID);
}
//---------------------------------------------------------------------------
// Lecture fichier calibration
void OuvrirFichierCalibration() {
  if (SPIFFS.exists(filecalibration)) {
    // printf("filecalibration present\n");
    File f = SPIFFS.open(filecalibration, "r");
    for (int i = 0; i < 2; i++) { //Read
      String s = f.readStringUntil('\n');
      CoeffTension[i] = s.toFloat();
    }
    f.close();
  }
  else {
    printf("Creating Data File:%s\n",filecalibration); // valeur par defaut
    CoeffTension[0] = CoeffTensionDefaut;
    CoeffTension[1] = CoeffTensionDefaut;
    Recordcalib();
  }
  // printf("Coeff T Batterie = %d, Coeff T PanSol = %d\n",CoeffTension[0],CoeffTension[1]);
}
//---------------------------------------------------------------------------
// enregistrer fichier calibration en SPIFFS
void Recordcalib() {
  File f = SPIFFS.open(filecalibration, "w");
  f.println(CoeffTension[0]);
  f.println(CoeffTension[1]);
  f.close();
}
//---------------------------------------------------------------------------
void mesureTension(){
  Vbatt = map(moyenneAnalogique(PinBattProc),0,4095,0,CoeffTension[0]);
  Vpansol = map(moyenneAnalogique(PinPannSol),0,4095,0,CoeffTension[1]);
  Serial.print("Coeff T Batterie = ")	  , Serial.print(CoeffTension[0]);
  Serial.print(", Coeff T PanSol = ")	, Serial.println(CoeffTension[1]);
  Serial.print("tension batterie:"),Serial.println(Vbatt);
  Serial.print("tension pansol  :"),Serial.println(Vpansol);
  }
//---------------------------------------------------------------------------
int moyenneAnalogique(int Pin) {	// calcul moyenne 10 mesures consécutives
  int moyenne = 0;
  for (int j = 0; j < 10; j++) {
    moyenne += analogRead(Pin);
    delay(1);
  }
  moyenne /= 10;
  return moyenne;
}
//---------------------------------------------------------------------------
void MajSoft() {
  // cherche si mise à jour dispo et maj
  if (connectWIFI()) {
    Serial.print(F("Vérification si une mise à jour est disponible ? "));
    // FlagMajSoft = false;
    String Fichier = "http://";
    Fichier += tempServer;
    Fichier += "/bin/" ;
    Fichier += soft;
    Fichier += String (ver + 1); // cherche version actuelle + 1
    Fichier += ".bin";
    Serial.println(Fichier);
    t_httpUpdate_return ret = ESPhttpUpdate.update(Fichier);
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        Serial.println(F("Pas de mise à jour disponible !"));
        break;
      case HTTP_UPDATE_NO_UPDATES:
        Serial.println(F("HTTP_UPDATE_NO_UPDATES"));
        break;
      case HTTP_UPDATE_OK:
        Serial.println(F("HTTP_UPDATE_OK"));
        break;
    }
  }
}
//---------------------------------------------------------------------------
void PrintEEPROM(){
  Serial.print("magic:"),Serial.println(config.magic);
  Serial.print("mqttServer:"),Serial.println(config.mqttServer);
  Serial.print("mqttUserName:"),Serial.println(config.mqttUserName);
  Serial.print("mqttPass:"),Serial.println(config.mqttPass);
  Serial.print("sendTopic:"),Serial.println(config.sendTopic);
  Serial.print("receiveTopic:"),Serial.println(config.receiveTopic);
  Serial.print("permanentTopic:"),Serial.println(config.permanentTopic);
  Serial.print("mqttPort:"),Serial.println(config.mqttPort);
  Serial.print("rain_to_mm:"),Serial.println(config.rain_to_mm);
  Serial.print("interval:"),Serial.println(config.interval);
}
//---------------------------------------------------------------------------
void sauvConfig() { // sauve configuration en EEPROM
  EEPROM.begin(512);
  EEPROM.put(confign, config);
  EEPROM.commit();
  EEPROM.end();
}
//---------------------------------------------------------------------------
void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println(F("- failed to open directory"));
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(F(" - not a directory"));
    return;
  }

  File file = root.openNextFile();
  Sbidon = "";
  while (file) {
    if (file.isDirectory()) {
      Serial.print(F("  DIR : "));
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print(F("  FILE: "));
      Serial.print(file.name());
      Serial.print(F("\tSIZE: "));
      Serial.println(file.size());
      
      Sbidon += F("  FILE: ");
      Sbidon += file.name();
      Sbidon += F("\tSIZE: ");
      Sbidon += file.size();
      Sbidon += '\n';
    }
    file = root.openNextFile();
  }
  file.close();
}