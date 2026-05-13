#include <Wire.h> // includem libraria Wire.h pentru a comunica cu modulul giroscop prin protocolul I2C
#include <EEPROM.h> // includem libraria EEPROM.h pentru a memora informatii in libraria EEPROM
// declararea variabilelor globale
byte ultimul_canal_1, ultimul_canal_2, ultimul_canal_3, ultimul_canal_4;
byte lowByte, highByte, tipul, adresa_giroscop, flag_eroare, clockspeed_ok;
byte canal_1_atribuit, canal_2_atribuit, canal_3_atribuit, canal_4_atribuit;
byte axa_roll, axa_pitch, axa_yaw;
byte byte_semnale_reciver, gyro_check_byte;
volatile int reciver_canal_1, reciver_canal_2, reciver_canal_3, reciver_canal_4;
int reciver_semnal_central_1, reciver_semnal_central_2, reciver_semnal_central_3, reciver_semnal_central_4;
int max_semnal_canal_1, max_semnal_canal_2, max_semnal_canal_3, max_semnal_canal_4;
int min_semnal_canal_1, min_semnal_canal_2, min_semnal_canal_3, min_semnal_canal_4;
int adresa, index;
unsigned long timp, timp_1, timp_2, timp_3, timp_4, timp_curent_intreruperi;
float giro_pitch, giro_roll, giro_yaw;
// functia de setup unde setam pinii care vor declansa intreruperi
// tot aici, se seteaza microcontroler-ul ca fiind Master, pentru protocolul I2C
void setup(){
  // registrul PCICR controleaza 3 porturi de pini care pot declansa intreruperi
  // aceste 3 porturi/grupui sunt controlate de alti 3 registri (PCMSK0, PCMSK1, PCMSK2)
  PCICR |= B00000001; // setam PCICR pentru a activa registrul PCMSK0, care controleaza pinii 8,9,10 si 11
  PCMSK0 |= B00000001; // setam registrul PCINT0 (pinul 8) sa declanseze o intrerupere
  PCMSK0 |= B00000010; // setam registrul PCINT1 (pinul 9) sa declanseze o intrerupere
  PCMSK0 |= B00000100; // setam registrul PCINT2 (pinul 10) sa declanseze o intrerupere
  PCMSK0 |= B00010000; // setam registrul PCINT3 (pinul 11) sa declanseze o intrerupere
  Wire.begin(); // Incepem comunicarea prin I2C, cu microcontroler-ul ca fiind Master
  Serial.begin(57600); // incepem conexiunea cu minitorul serial @ 57600bps
  delay(400); // dam timp giroscopului si reciverului sa porneasca 
}
// incepem programul principal, o functie care se va tot repeta 
// aici vor avea loc testele si memorarea variabilelor necesare
void loop(){
  Serial.println(F("Incepe Testarea sistemului"));
  Serial.println(F("Se verifica daca clock-ul din protocolul I2C este setat la viteza corespunzatoare"));
  delay(1000);
  // setam viteza de comunicare a I2C-ului la 400 kHz (modul rapid)
  // ca setare initiala, aceasta viteza este de 100kHz (modul minim)
  Wire.setClock(400000); // analog, se poate seta si prin schimbarea registrului TWBR = 12
  Serial.print(F("Se verifca semnalele reciverului"));
  delay(1000);
  semnale_reciver(); // verificam daca exista semnal de la reciver, prin toate cele 4 canale de comunicare
  if(flag_eroare == 0){ // daca nu am avut eroare anterior
    delay(1000);
    Serial.println(F("Verificam valorile, cu manetele in pozitie centrala"));
    delay(10000); // asteptam cateva secunde pentru a pune matele in pozitie centrala 
	// vrem sa memoram valorile semnalelor cand manetele sunt in pozitie centrala
	// aceste valori vor fi folosite ca repere in procesul de configurare dar si in functionalitatea dronei
	verificare_pozitie_centrala();
  }
  // vom verifica, in particular, semnale pentru miscarile manetelor
  if(flag_eroare == 0){ // daca nu am avut eroare anterior 
	// verificam miscarea throttle-ului 
    Serial.println(F("Se va misca maneta throttle-ului (stanga) si inapoi in centru"));
    verificare_semnale_reciver(1); // trimitem valoare 1, ca valoare de referinta
    Serial.print(F("Throttle este connectat la pinul digital: "));
    Serial.println((canal_3_atribuit & 0b00000111) + 7); // afisam pinul digital corespunzator
	// pentru a trece la urmatoarea configurare, vrem sa incepem cu manetele din pozitie centrala
    manete_pozitie_centrala(); // apelam functia care verifica semnalele pentru pozitie centrala
  }
  if(flag_eroare == 0){ // daca nu am avut eroare anterior 
	// verificam miscarea pentru roll
    Serial.println(F("Se va misca maneta pentru roll (dreapta) spre dreapta si inapoi in centru"));
    verificare_semnale_reciver(2); // trimitem valoare 2, ca valoare de referinta
    Serial.print(F("Roll-ul este connectat la pinul digital: ")); 
    Serial.println((canal_1_atribuit & 0b00000111) + 7); // afisam pinul digital corespunzator
    manete_pozitie_centrala(); // apelam functia care verifica semnalele pentru pozitie centrala
  }
  if(flag_eroare == 0){ // daca nu am avut eroare anterior 
	// verificam miscarea pentru pitch
    Serial.println(F("Se va misca maneta pentru pitch (dreapta) in jos si inapoi in centru"));
    verificare_semnale_reciver(3); // trimitem valoare 3, ca valoare de referinta
    Serial.print(F("Pitch-ul este connectat la pinul digital: "));
    Serial.println((canal_2_atribuit & 0b00000111) + 7); // afisam pinul digital corespunzator
    manete_pozitie_centrala(); // apelam functia care verifica semnalele pentru pozitie centrala
  }
  if(flag_eroare == 0){ // daca nu am avut eroare anterior 
	// verificam miscarea pentru yaw
    Serial.println(F("Se va misca maneta pentru yaw (stanga) in dreapta si inapoi in centru"));
    verificare_semnale_reciver(4); // trimitem valoare 3, ca valoare de referinta
    Serial.print(F("Yaw-ul este connectat la pinul digital: "));
    Serial.println((canal_4_atribuit & 0b00000111) + 7); // afisam pinul digital corespunzator
    manete_pozitie_centrala(); // apelam functia care verifica semnalele pentru pozitie centrala
  }
  // vom verifica valoarea semnalelor in pozitiile sale extreme
  if(flag_eroare == 0){ // daca nu am avut eroare anterior 
    Serial.println(F("Se vor misca ambele manete, simultan, spre punctele extreme"));
    valori_min_max(); // cautam valorile minime si maxime ale canalelor reciverului.
    Serial.println(F("Valorile maxime, minime si centrale:")); // afisam valorie
    Serial.print(F("Digital input 08 values:"));
    Serial.print(min_semnal_canal_1);
    Serial.print(reciver_semnal_central_1);
    Serial.print(max_semnal_canal_1);
    Serial.print(F("Digital input 09 values:"));
    Serial.print(min_semnal_canal_2);
    Serial.print(reciver_semnal_central_2);
    Serial.print(max_semnal_canal_2);
    Serial.print(F("Digital input 10 values:"));
    Serial.print(min_semnal_canal_3);
    Serial.print(reciver_semnal_central_3);
    Serial.print(max_semnal_canal_3);
    Serial.print(F("Digital input 11 values:"));
    Serial.print(min_semnal_canal_4);
    Serial.print(reciver_semnal_central_4);
    Serial.print(max_semnal_canal_4);
	// cand ne vom opri din masurat, vom aduce manetele in pozitie centrala 
	// si vom introduce valoare 'c' in monitorul serial, pentru a continua configurarea
    continuare_configurare();
  }
  if(flag_eroare == 0){ // daca nu am avut eroare anterior 
    //Verificare model giroscop
    Serial.println(F("Cuatam modulul giroscop si accelerometru"));
    Serial.println(F("Cautam modelul MPU-6050 la adresa 0x68/104"));
    delay(1000);
    if(cautare_giroscop(0x68, 0x75) == 0x68){ //verificam daca giroscopul exista
      Serial.println(F("MPU-6050 a fost gasit la adresa: 0x68"));
      adresa_giroscop = 0x68; // completam cu adresa giroscopului
	  tipul = 1; //completam si o variabila ce ne va fi utila ulterior
    }
    else{ // cazul in care giroscopul nu a fost gasit
      Serial.println(F("EROARE: NU A FOST GASIT MODULUL GIROSCOP SI ACCELEROMETRU"));
      flag_eroare = 1;
    }
    if(flag_eroare == 0){ // cazul in care a fost gasit
      delay(3000);
      Serial.println(F("Vom incepe Setup-ul modulului giroscop si accelerometru"));
      start_giroscop(); //functie care incepe setup-ul giroscopului
    }
  }
  // daca giroscopul a fost gasit, putem incepe calibrarea acestuia
  if(flag_eroare == 0){ // daca nu exista eroare
    // facem cateva verificari pentru fiecare axa si memoram rezultatele
    Serial.println(F("Se ridica partea stanga a dronei la un unghi de aproximativ 45 de grade"));
    
    verificare_axe_giroscop(1); // se verifica miscarea
    if(flag_eroare == 0){ // daca nu exista eroare
      Serial.print(F("Unghiul detectat a fost = "));
      Serial.println(axa_roll & 0b00000011);
	  //verificam cazul in care axele sunt inversate, care poate aparea din costructia giroscopului
      if(axa_roll & 0b10000000)Serial.println(F("Axele sunt inversate"));
      Serial.println(F("Asezati drona in pozitia initiala"));
      continuare_configurare(); // apelam functia pentru continuarea procesului
      Serial.println(F("Ridicati partea din fata dronei la un unghi de aproximativ 45 de grade"));
      verificare_axe_giroscop(2); // se verifica miscarea
    }
    if(flag_eroare == 0){ // daca nu exista eroare
      Serial.print(F("Unghiul detectat a fost = "));
      Serial.println(axa_pitch & 0b00000011);
      if(axa_pitch & 0b10000000)Serial.println(F("Axele sunt inversate"));
      Serial.println(F("Asezati drona in pozitia initiala"));
      continuare_configurare(); // apelam functia pentru continuarea procesului 
      Serial.println(F("Rotiti drona in partea sa dreapta la un unghi de aproximativ 45 de grade"));
      verificare_axe_giroscop(3); // se verifica miscarea
    }
    if(flag_eroare == 0){ // daca nu exista eroare
      Serial.print(F("Unghiul detectat a fost = "));
      Serial.println(axa_yaw & 0b00000011);
      if(axa_yaw & 0b10000000)Serial.println(F("Axele sunt inversate"));
      Serial.println(F("Asezati drona in pozitia initiala"));
      continuare_configurare(); // apelam functia pentru continuarea procesului 
    }
  }
  if(flag_eroare == 0){
	Serial.println(F("Ultima verificare:"));
    delay(1500);
    if(byte_semnale_reciver == 0b00001111){
      Serial.println(F("Semnalele reciverului sunt ok!"));
    }
    else{
      Serial.println(F("EROARE: SEMNALELE RECIVER"));
      flag_eroare = 1;
    }
    delay(1000);
    if(gyro_check_byte == 0b00000111){
      Serial.println(F("Axele giroscopului sunt ok!"));
    }
    else{
      Serial.println(F("EROARE: AXELE GIROSCOPULUI"));
      flag_eroare = 1;
    }
  }     
  if(flag_eroare == 0){// verificam daca exista eroare
	// retinem datele in memoria EEPROM
    Serial.println(F("Se memoreaza datele"));
    delay(2000);
    EEPROM.write(0, reciver_semnal_central_1 & 0b11111111);
    EEPROM.write(1, reciver_semnal_central_1 >> 8);
    EEPROM.write(2, reciver_semnal_central_2 & 0b11111111);
    EEPROM.write(3, reciver_semnal_central_2 >> 8);
    EEPROM.write(4, reciver_semnal_central_3 & 0b11111111);
    EEPROM.write(5, reciver_semnal_central_3 >> 8);
    EEPROM.write(6, reciver_semnal_central_4 & 0b11111111);
    EEPROM.write(7, reciver_semnal_central_4 >> 8);
    EEPROM.write(8, max_semnal_canal_1 & 0b11111111);
    EEPROM.write(9, max_semnal_canal_1 >> 8);
    EEPROM.write(10, max_semnal_canal_2 & 0b11111111);
    EEPROM.write(11, max_semnal_canal_2 >> 8);
    EEPROM.write(12, max_semnal_canal_3 & 0b11111111);
    EEPROM.write(13, max_semnal_canal_3 >> 8);
    EEPROM.write(14, max_semnal_canal_4 & 0b11111111);
    EEPROM.write(15, max_semnal_canal_4 >> 8);
    EEPROM.write(16, min_semnal_canal_1 & 0b11111111);
    EEPROM.write(17, min_semnal_canal_1 >> 8);
    EEPROM.write(18, min_semnal_canal_2 & 0b11111111);
    EEPROM.write(19, min_semnal_canal_2 >> 8);
    EEPROM.write(20, min_semnal_canal_3 & 0b11111111);
    EEPROM.write(21, min_semnal_canal_3 >> 8);
    EEPROM.write(22, min_semnal_canal_4 & 0b11111111);
    EEPROM.write(23, min_semnal_canal_4 >> 8);
    EEPROM.write(24, canal_1_atribuit);
    EEPROM.write(25, canal_2_atribuit);
    EEPROM.write(26, canal_3_atribuit);
    EEPROM.write(27, canal_4_atribuit);
    EEPROM.write(28, axa_roll);
    EEPROM.write(29, axa_pitch);
    EEPROM.write(30, axa_yaw);
    EEPROM.write(31, tipul);
    EEPROM.write(32, adresa_giroscop);
    Serial.println(F("Finalizare!"));
  while(1);
  }
}
// FUNCTII PENTRU GIROSCOP
// cautam modulul giroscop si verificam registrul Who_am_I
// registrul who_am_i este folosit pentru a verifica identitatea giroscopului.
byte cautare_giroscop(int adresa_giroscop, int who_am_i){
  // incepem o transmitere de date dintre Master(Arduino) si Slave( Giroscop)
  // adresa giroscopului este data de registrul: adresa_giroscop
  Wire.beginTransmission(adresa_giroscop);
  Wire.write(who_am_i); // adaugam in coada de bytes, byte-ul: who_am_i
  Wire.endTransmission(); // oprim transmiterea de date. Trimitem coada de bytes creata anterior
  Wire.requestFrom(adresa_giroscop, 1); // se face un cerere de un singur byte (param: quantity=1)
  timp = millis() + 100; // dam la dispozitie un timp pentru procesul buclei: while()
  // Wire.available() returneaza numarul de bytes disponibili pentru citire
  // giroscop-ul poate sa trimita mai putini bytes, decat am cerut
  // vom citi, cat timp exista cel putin un byte.
  while(Wire.available() < 1 && timp > millis()); // ateptam pana cand byte-ul este primit
  lowByte = Wire.read(); // citim un bytes
  adresa = adresa_giroscop; // completam variabila globala ce memoreaza adresa
  return lowByte; // returnam adresa giroscopului, 0x68 
}
void start_giroscop(){
  if(tipul == 1){ // in cazul in care ne aflam pe modelul corespunzator
	Wire.beginTransmission(adresa); // incepem comunicarea cu giroscopul
	Wire.write(0x6B); // registrul 107 (PWR_MGMT_1) permite configurarea modulului de alimentare si sursa ceasului
	//in prima faza, giroscopul vine mod activat, sleep mode
	//acesta va trebui schimbat, pentru a putea porni giroscopul
	Wire.write(0x00); // schimbam valoarea registrului PWR_MGMT_1 la 0 pentru a porni giroscopul
	Wire.endTransmission(); // oprim transmiterea de date si trimitem registrii
	Wire.beginTransmission(adresa); // incepem comunicarea cu giroscopul
	Wire.write(0x6B); // scriem registrul PWR_MGMT_1
	Wire.endTransmission(); // oprim transmiterea de date
	Wire.requestFrom(adresa, 1); // solicitam un byte de la giroscop
	while(Wire.available() < 1); // asteptam pana cand un byte este primit
	Serial.print(F("Register 0x6B is set to:")); // verificam prin acel byte, setarea a avut loc
	Serial.println(Wire.read(),BIN); // BIN este o constanta ce ne ajuta sa printam rezultatul in baza 2
	Wire.beginTransmission(adresa); // incepem comunicarea cu giroscopul
	// registrul GYRO_CONFIG (27 sau 0x1B) este folosit pentru a declansa autotestarea giroscopului
	// astfel, se permite testare portiunilor mecanice sau electrice
	// exista 3 registri (XG_ST,YG_ST,ZG_ST) care pot oferi infromatii despre axele giroscopului
	// acesti registri pot oferi date independet sau simultan
	Wire.write(0x1B); // anuntam ca vrem sa scriem in registrul GYRO_CONFIG
	Wire.write(0x08); // setam bitii registrului cu 00001000 (1 pentru bitul FS_SEL=Full scale range)(500dps)- degree per second
	// astfel, selectam gama completa de valori ale giroscopului
	Wire.endTransmission(); // oprim transmiterea de date
	Wire.beginTransmission(adresa); // incepem comunicarea cu giroscopul)
	Wire.write(0x1B); // vrem sa citim date de la registrul GYRO_CONFIG
	Wire.endTransmission(); // oprim transmiterea de date
	Wire.requestFrom(adresa, 1); // solicitam un byte de la giroscop
	while(Wire.available() < 1); // ateptam pana cand byte-ul este primit
	Serial.print(F("Register 0x1B is set to:"));
	Serial.println(Wire.read(),BIN); // verificam byte-ul
  }
}
// aceasta este principala functie care citeste date de la axele OX, OY, OZ ale giroscopului
// exista o proprietate de autoincrementare a registrilor, pentru a citi datele in acelasi moment de timp
// datele sunt stocate in 16 biti (2 registri de cate 8 biti)
// acesti 16 biti se obtini prin combinarea a 2 registri de 8 biti 
// pentru a extrage datele complete, bitii trebuie siftati pe pozitiile corespunzatoare
void semnale_giroscop(){
  if(tipul == 1){ // daca suntem pe tipul corespunzator
    Wire.beginTransmission(adresa); // incepem comunicarea cu giroscopul
    Wire.write(0x43); // registrul Gyroscope Measurements (0x43, 67) stocheaza cele mai recente masuratori ale giroscopului
    Wire.endTransmission(); // oprim transmiterea de date
    Wire.requestFrom(adresa,6); // solicitam giroscopului 6 bytes
    while(Wire.available() < 6); // asteptam pana cand cei 6 byets sunt primiti
	// pentru a avea datele complete, trebuie sa aplicam cateva operatii pe biti
	// prima citire este pentru high byte-ul care se sifteaza cu 8 bites la stanga
	// peste spatiul creat se copiaza low byte-ul
    giro_roll=Wire.read()<<8|Wire.read(); //Citim byte-ul high si cel low pentru a extrage date pentru axa OX a giroscopului
    giro_pitch=Wire.read()<<8|Wire.read(); //Citim byte-ul high si cel low pentru a extrage date pentru axa OY a giroscopului
    giro_yaw=Wire.read()<<8|Wire.read(); //Citim byte-ul high si cel low pentru a extrage date pentru axa OZ a giroscopului
  }
}
//verificam cum sunt primite datele pentru axele giroscopului, raportat la miscarea dronei
void verificare_axe_giroscop(byte miscare_drona){
  byte miscare_axa = 0;
  float giro_roll_unghi, giro_pitch_unghi, giro_yaw_unghi; // initializam variabile pentru unghiurile corespunzatoare
  //initializam valorile cu 0
  giro_roll_unghi = 0;
  giro_pitch_unghi = 0;
  giro_yaw_unghi = 0;
  //apelam functia pentru a cere date de la giroscop
  semnale_giroscop();
  timp = millis() + 10000; //punem o limita de timp pentru procesul buclei: while()
  // acest bloc de cod ruleaza pana cand datele unghiulare sunt schimbate sau cand timpul expira
  // este de preferat ca miscarile sa corespunda cu cerinta din mesajul anterior
  while(timp > millis() && giro_roll_unghi > -30 && giro_roll_unghi < 30 && giro_pitch_unghi > -30 && giro_pitch_unghi < 30 && giro_yaw_unghi > -30 && giro_yaw_unghi < 30){
	semnale_giroscop(); // se fac apeluri pentru a prelua date
	//LSB= least significant bit unit to the related number of radians per second
	// acum trebuie sa calculam unghiul parcurs, pentru o fiecare miscare a dronei
	// viteza de calcul este de 250(Hz) = 0.004(s)
	// conform manualului de utilizare a modulului giroscop, pentru a putea folosi date primite de la registri
	// vor trebui facute cateva conversii
	// pentru ca am configurat giroscopul la 500dps(degree per second/ grade pe secunda), avem o sensibilitate de 65.5 LBS/degree/s
	// LSB reprezinta cel mai semnificativ bite reprezentativ pentru numarul de grade pe secunda
	// folosind formula din manualul de utilizare, giro_output * 1/65.5 * 0.004 = giro_output * 0.0000611 
	// astfel se obtine viteza unghiulara care este adunata in variabila ce retine dinstanta parcursa in functie de viteza
    giro_roll_unghi += giro_roll * 0.0000611;  
    giro_pitch_unghi += giro_pitch * 0.0000611;
    giro_yaw_unghi += giro_yaw * 0.0000611;
    delayMicroseconds(37000); // oferim un timp pentru ridicarea dronei
  }
  // dupa ce a fost simtita o miscare peste valoarea limita, se cauta axa corespunzatoare
  // atribuim axei miscate functia corespunzatoare (roll, pitch sau yaw), in functie de datele unghiulare
  if((giro_roll_unghi < -30 || giro_roll_unghi > 30) && giro_pitch_unghi > -30 && giro_pitch_unghi < 30 && giro_yaw_unghi > -30 && giro_yaw_unghi < 30){
    gyro_check_byte |= 0b00000001;
    if(giro_roll_unghi < 0)miscare_axa = 0b10000001;
    else miscare_axa = 0b00000001;
  }
  if((giro_pitch_unghi < -30 || giro_pitch_unghi > 30) && giro_roll_unghi > -30 && giro_roll_unghi < 30 && giro_yaw_unghi > -30 && giro_yaw_unghi < 30){
    gyro_check_byte |= 0b00000010;
    if(giro_pitch_unghi < 0)miscare_axa = 0b10000010;
    else miscare_axa = 0b00000010;
  }
  if((giro_yaw_unghi < -30 || giro_yaw_unghi > 30) && giro_roll_unghi > -30 && giro_roll_unghi < 30 && giro_pitch_unghi > -30 && giro_pitch_unghi < 30){
    gyro_check_byte |= 0b00000100;
    if(giro_yaw_unghi < 0)miscare_axa = 0b10000011;
    else miscare_axa = 0b00000011;
  }
  if(miscare_axa == 0){ //caz in care nu a fost detectata miscarea pe o axa
    flag_eroare = 1;
    Serial.println(F("EROARE: NU A FOST DETECTATA MISCAREA"));
  }
  else{ //daca a fost detectata miscare, in functie memoram axa corespunzatoare
	if(miscare_drona == 1)axa_roll = miscare_axa;
	if(miscare_drona == 2)axa_pitch = miscare_axa;
	if(miscare_drona == 3)axa_yaw = miscare_axa;
  }
}
// FUNCTII PENTRU SEMNALELE RECIVER-ULUI
// verificam daca valoarea de intrare se schimba intr-un anumit timp
void verificare_semnale_reciver(byte miscare_maneta){
  byte valoare_semnal = 0; // variabila care ne ajuta sa retinem canalul de comunicare
  bool primit_semnal = false; // variabila care memoreaza daca am primit semnal
  int pulsatie_semnal_reciver; // variabila care memoreaza valoarea semnalului
  timp = millis() + 30000; // adaugam un timp pentru executie
  while(timp > millis() && primit_semnal == false){ //cat timp nu am primit semnal
    delay(250);
	// valorile 1750 si 1250 reprezinta intervalul in care manetele sunt in pozitie centrala
	// in momentul in care, semnalele depasesc aceste valori, stim ca a avut loc un procesul
	// in functie de caz, retinem ce canal a receptat semnalul
    if(reciver_canal_1 > 1750 || reciver_canal_1 < 1250){
      valoare_semnal = 1;
	  primit_semnal = true;
      byte_semnale_reciver |= 0b00000001;
      pulsatie_semnal_reciver = reciver_canal_1;
    }
    if(reciver_canal_2 > 1750 || reciver_canal_2 < 1250){
      valoare_semnal = 2;
	  primit_semnal = true;
      byte_semnale_reciver |= 0b00000010;
      pulsatie_semnal_reciver = reciver_canal_2;
    }
    if(reciver_canal_3 > 1750 || reciver_canal_3 < 1250){
      valoare_semnal = 3;
	  primit_semnal = true;
      byte_semnale_reciver |= 0b00000100;
      pulsatie_semnal_reciver = reciver_canal_3;
    }
    if(reciver_canal_4 > 1750 || reciver_canal_4 < 1250){
      valoare_semnal = 4;
	  primit_semnal = true;
      byte_semnale_reciver |= 0b00001000;
      pulsatie_semnal_reciver = reciver_canal_4;
    } 
  }
  if(primit_semnal == false){ // verificam daca au fost detectate semnale
    flag_eroare = 1;
    Serial.println(F("EROARE: NU AU FOST DETECTATE SEMNALE DE LA RECIVER"));
  }
  // in cazul in care a fost detectat semnal
  // atribuim, in functie de preferinta, miscarea pe canalul de comunicare
  else{
    if(miscare_maneta == 1){
      canal_3_atribuit = valoare_semnal;
      if(pulsatie_semnal_reciver < 1250)canal_3_atribuit += 0b10000000;
    }
    if(miscare_maneta == 2){
      canal_1_atribuit = valoare_semnal;
      if(pulsatie_semnal_reciver < 1250)canal_1_atribuit += 0b10000000;
    }
    if(miscare_maneta == 3){
      canal_2_atribuit = valoare_semnal;
      if(pulsatie_semnal_reciver < 1250)canal_2_atribuit += 0b10000000;
    }
    if(miscare_maneta == 4){
      canal_4_atribuit = valoare_semnal;
      if(pulsatie_semnal_reciver < 1250)canal_4_atribuit += 0b10000000;
    }
  }
}
// aceasta functie este folosita pentru a ne ajuta in pasii de configurare
void continuare_configurare(){ 
  Serial.print(F("Introduceti de la tastatura litera 'c' pentru a continua configurarea:"));
  byte data;
  if(Serial.available() > 0){
	while(data != 'c'){
	  data = Serial.read();  //citeste valoarea pentru a continua.
	  if(data == 'c')
		manete_pozitie_centrala();                                                          
	}
  }else{
	  Serial.print(F("EROARE: Monitorul serial nu este deschis"));
  }
}
// verificam daca manetele sunt in pozitie centrala
void manete_pozitie_centrala(){
  byte zero = 0; // byte-ul care memoreaza daca sunt in pozitie centrala
  const int tol = 20; // toleranta, in cazul in care manetele sunt cu aproximatie, in pozitie centrala
  while(zero < 15){ // 15 = 0b00001111;
    if(reciver_canal_1 < reciver_semnal_central_1 + tol && reciver_canal_1 > reciver_semnal_central_1 - tol)zero |= 0b00000001;
    if(reciver_canal_2 < reciver_semnal_central_2 + tol && reciver_canal_2 > reciver_semnal_central_2 - tol)zero |= 0b00000010;
    if(reciver_canal_3 < reciver_semnal_central_3 + tol && reciver_canal_3 > reciver_semnal_central_3 - tol)zero |= 0b00000100;
    if(reciver_canal_4 < reciver_semnal_central_4 + tol && reciver_canal_4 > reciver_semnal_central_4 - tol)zero |= 0b00001000;
    delay(100);
  }
}
// functie care verifica existenta semnalelor primite de la reciver
// variabilele sunt actualizate mereu, prin functia care trateaza intreruperile (ISR())
void semnale_reciver(){
  byte byte_existenta_semnale_reciver = 0; // variabila care memoram daca exista semnal
  timp = millis() + 10000; // dam un timp buclei while, pentru a nu o repeta nesfarsit
  while(timp > millis() && byte_existenta_semnale_reciver < 15){ //15 = 0b00001111
	// valorile semnalului ar trebui sa fie cuprinse intre 900 si 2100
	// daca semnalul este valid, memoram o adresa de memorie corespunzatoare in variabila declarata mai sus
    if(reciver_canal_1 < 2100 && reciver_canal_1 > 900)byte_existenta_semnale_reciver |= 0b00000001;
    if(reciver_canal_2 < 2100 && reciver_canal_2 > 900)byte_existenta_semnale_reciver |= 0b00000010;
    if(reciver_canal_3 < 2100 && reciver_canal_3 > 900)byte_existenta_semnale_reciver |= 0b00000100;
    if(reciver_canal_4 < 2100 && reciver_canal_4 > 900)byte_existenta_semnale_reciver |= 0b00001000;
    delay(500); // intarziem procesul, pentru a oferi timp procesului
  }
  if(byte_existenta_semnale_reciver == 0){ //verificam daca exista semnale
    flag_eroare = 1; // tratam eroarea 
    Serial.println(F("EROARE: NU AU FOST DETECTATE SEMNALE)"));
  }
}
void verificare_pozitie_centrala(){
	// memoram in variabile semnalele pentru pozitia centrala
    reciver_semnal_central_1 = reciver_canal_1;
    reciver_semnal_central_2 = reciver_canal_2;
    reciver_semnal_central_3 = reciver_canal_3;
    reciver_semnal_central_4 = reciver_canal_4;
	// le afisam pe monitorul serial
    Serial.print(F("Pinul 8 = "));
    Serial.println(reciver_canal_1);
    Serial.print(F("Pinul 9 = "));
    Serial.println(reciver_canal_2);
    Serial.print(F("Pinul 10 = "));
    Serial.println(reciver_canal_3);
    Serial.print(F("Pinul 11 = "));
    Serial.println(reciver_canal_4);
    Serial.println(F(""));
}
// functie care memoreaza valorile maxime si minime ale pozitiei manetelor
void valori_min_max(){ 
  byte zero = 0; // byte folosit daca a fost trimis semnal pe un anumit canal
  const int tol = 10; // toleranta pentru pozitie centrala
  // dam cateva valori de start pentru semnalele minime
  min_semnal_canal_1 = reciver_canal_1;
  min_semnal_canal_2 = reciver_canal_2;
  min_semnal_canal_3 = reciver_canal_3;
  min_semnal_canal_4 = reciver_canal_4;
  delay(250);
  Serial.println(F("Incepem masurarea valorilor extreme...."));
  while(zero < 15){ // cat timp au fost detectate toate semnalele (15 = 0b00001111)
    if(reciver_canal_1 < min_semnal_canal_1)min_semnal_canal_1 = reciver_canal_1;
    if(reciver_canal_2 < min_semnal_canal_2)min_semnal_canal_2 = reciver_canal_2;
    if(reciver_canal_3 < min_semnal_canal_3)min_semnal_canal_3 = reciver_canal_3;
    if(reciver_canal_4 < min_semnal_canal_4)min_semnal_canal_4 = reciver_canal_4;
    if(reciver_canal_1 > max_semnal_canal_1)max_semnal_canal_1 = reciver_canal_1;
    if(reciver_canal_2 > max_semnal_canal_2)max_semnal_canal_2 = reciver_canal_2;
    if(reciver_canal_3 > max_semnal_canal_3)max_semnal_canal_3 = reciver_canal_3;
    if(reciver_canal_4 > max_semnal_canal_4)max_semnal_canal_4 = reciver_canal_4;
	// conditiile de oprire a procesului de masurare
	// cand manetele sunt asezate din nou in pozitie cenrala
	if(reciver_canal_1 < reciver_semnal_central_1 + tol && reciver_canal_1 > reciver_semnal_central_1 - tol)zero |= 0b00000001;
    if(reciver_canal_2 < reciver_semnal_central_2 + tol && reciver_canal_2 > reciver_semnal_central_2 - tol)zero |= 0b00000010;
    if(reciver_canal_3 < reciver_semnal_central_3 + tol && reciver_canal_3 > reciver_semnal_central_3 - tol)zero |= 0b00000100;
    if(reciver_canal_4 < reciver_semnal_central_4 + tol && reciver_canal_4 > reciver_semnal_central_4 - tol)zero |= 0b00001000;
    delay(100);
  }
}
// aceasta functie este apelata de fiecare data cand valorile de intrare de la pinii 8,9,10 sau 11 se schimba
ISR(PCINT0_vect){ // rutina intreruperilor
  timp_curent_intreruperi = micros(); //memoram timpul curent al microcontroler-ului
  // PINB este registrul care controleaza portul piniilor de la 8 la 13
  // verificam de unde a fost declansata o intrerupere
  // canal 1
  if(PINB & B00000001){ // verificam daca avem intrerupere pe pinul 8
	// variabila care ne spune ca a inceput intreruperea
    if(ultimul_canal_1 == 0){ // verificam daca nu exista intrerupere
      ultimul_canal_1 = 1; // schimbam valoarea spunand ca avem intrerupere
      timp_1 = timp_curent_intreruperi; //setam timp_1 cu timp_curent_intreruperi
    }
  }
  else if(ultimul_canal_1 == 1){ // verificam daca exista intrerupere
    ultimul_canal_1 = 0; // schimbam valoare spunand ca nu mai avem intrerupere
    reciver_canal_1 = timp_curent_intreruperi - timp_1; // valoare canalului va fi egala cu timp_curent_intreruperi - timp_1
  }
  // canal 2
  if(PINB & B00000010 ){ // verificam daca avem intrerupere pe pinul 9
    if(ultimul_canal_2 == 0){ // verificam daca nu exista intrerupere
      ultimul_canal_2 = 1; // schimbam valoarea spunand ca avem intrerupere
      timp_2 = timp_curent_intreruperi; //setam timp_2 cu timp_curent_intreruperi
    }
  }
  else if(ultimul_canal_2 == 1){ // verificam daca exista intrerupere
    ultimul_canal_2 = 0; // schimbam valoare spunand ca nu mai avem intrerupere
    reciver_canal_2 = timp_curent_intreruperi - timp_2; // valoare canalului va fi egala cu timp_curent_intreruperi - timp_2
  }
  // canal 3
  if(PINB & B00000100 ){ // verificam daca avem intrerupere pe pinul 10
    if(ultimul_canal_3 == 0){ // verificam daca nu exista intrerupere
      ultimul_canal_3 = 1;  // schimbam valoarea spunand ca avem intrerupere
      timp_3 = timp_curent_intreruperi; //setam timp_3 cu timp_curent_intreruperi
    }
  }
  else if(ultimul_canal_3 == 1){ // verificam daca exista intrerupere
    ultimul_canal_3 = 0; // schimbam valoare spunand ca nu mai avem intrerupere
    reciver_canal_3 = timp_curent_intreruperi - timp_3; // valoare canalului va fi egala cu timp_curent_intreruperi - timp_3

  }
  // canal 3
  if(PINB & B00001000 ){ // verificam daca avem intrerupere pe pinul 11
    if(ultimul_canal_4 == 0){ // verificam daca nu exista intrerupere
      ultimul_canal_4 = 1; // schimbam valoarea spunand ca avem intrerupere
      timp_4 = timp_curent_intreruperi; //setam timp_4 cu timp_curent_intreruperi
    }
  }
  else if(ultimul_canal_4 == 1){ // verificam daca exista intrerupere
    ultimul_canal_4 = 0; // schimbam valoare spunand ca nu mai avem intrerupere
    reciver_canal_4 = timp_curent_intreruperi - timp_4; // valoare canalului va fi egala cu timp_curent_intreruperi - timp_4
  }
}
