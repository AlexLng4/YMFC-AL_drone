#include <Wire.h>// includem libraria Wire.h pentru a comunica cu modulul giroscop prin protocolul I2C
#include <EEPROM.h>// includem libraria EEPROM.h pentru a memora informatii in libraria EEPROM
#include <math.h>// includem libraria math.h pentru a folosi functii specifice in procesul de calibrare
// declararea variabilelor globale pentru controler
// parametrii pentru roll
float pid_p_roll = 1.3;// parametrul P pentru miscarea roll
float pid_i_roll = 0.04;// parametrul I pentru miscarea roll
float pid_d_roll = 18.0;// parametrul D pentru miscarea roll
int pid_max_roll = 400;// maximul valorii de iesire a controlerului
// parametrii pentru pitch
float pid_p_pitch = 1.3;// parametrul P pentru miscarea pitch
float pid_i_pitch = 0.04;// parametrul I pentru miscarea pitch
float pid_d_pitch = 18.0;// parametrul D pentru miscarea pitch
int pid_max_pitch = 400;// maximul valorii de iesire a controlerului
// parametrii pentru yaw
float pid_p_yaw = 4.0;// parametrul P pentru miscarea yaw
float pid_i_gain_yaw = 0.02;// parametrul I pentru miscarea yaw
float pid_d_yaw = 0.0;// parametrul D pentru miscarea yaw
int pid_max_yaw = 400;// maximul valorii de iesire a controlerului
// declararea variabilelor globale
byte ultimul_canal_1, ultimul_canal_2, ultimul_canal_3, ultimul_canal_4;//calculul valorilor de la reciver
byte vector_date_eeprom[33]; // vectorul de valori, stocate anterior in procesul de configurare
byte highByte, lowByte;// byete-ul cel mai semnificativ si cel mai nesemnificativ
// variabile care vor stoca valorile provenite de la reciver
volatile int reciver_canal_1, reciver_canal_2, reciver_canal_3, reciver_canal_4;// valorile venite de la reciverului
int esc_1, esc_2, esc_3, esc_4, throttle;// variabile pentru fiecare esc si throttle
int iterare, start, adresa_giroscop;
int receiver_input[5];
int temperatura;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;
long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_canal_1, timer_canal_2, timer_canal_3, timer_canal_4, esc_timer, esc_loop_timer;
unsigned long timp_1, timp_2, timp_3, timp_4, timp_curent_intreruperi;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double giro_axa_dif[4], giro_axa_med[4], giro_axa_val[4][2000];
float pid_eroare_temporara;
float pid_i_mem_roll, pid_roll_val_ideala, giro_roll_intrare, pid_iesire_roll, pid_ultima_eroare_d_roll;
float pid_i_mem_pitch, pid_pitch_val_ideala, giro_pitch_intrare, pid_iesire_pitch, pid_ultima_eroare_d_pitch;
float pid_i_mem_yaw, pid_yaw_val_ideala, giro_yaw_intrare, pid_iesire_yaw, pid_ultima_eroare_d_yaw;
float unghi_roll_acc, unghi_pitch_acc, unghi_pitch, unghi_roll;
boolean unghiurile_giroscopului_setate_flag;
void setup(){
  // compiem datele din eeprom intr-un vector ce au fost retinute la pasul de configurare
  // la fel si pentru adresa giroscopului
  copiere_eeprom();// facem asta pentru a avea acces mai usor la date
  Wire.begin();// incepem comunicarea prin I2C, cu microcontroler-ul ca fiind Master    
  // setam viteza de comunicare a I2C-ului la 400 kHz (modul rapid)
  // ca setare initiala, aceasta viteza este de 100kHz (modul minim)
  Wire.setClock(400000); // analog, se poate seta si prin schimbarea registrului TWBR = 12
  // Arduino (Atmega) are toti pinii declararti ca intrari
  // in schimb, trebuie sa declaram pinii pe care-i vrem ca iesiri
  pinMode(12, OUTPUT);// analog prin schimbarea valorii registrului DDRB |= B00110000
  pinMode(13, OUTPUT);
  // analog prin schimbarea valorii registrului DDRD |= B11110000, pentru fiecare
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  configurare_giroscop();// functie care configureaza registrii modulul giroscop si accelerometru     
  // senzorul giroscop, ca si orice alt senzor, are o anumita acuratete in citirea datelor 
  // iar datele furnizate reprezinta doar o aproximare a valorilor exacte
  // din acest motiv, pentru a micsora eroarea
  // este necesara calibrarea senzorului inainte de utilizarea datelor
  calibrare_giroscop();// vom citi mai multe date de la giroscop si vom face calibarea sa
  // registrul PCICR controleaza 3 porturi de pini care pot declansa intreruperi
  // aceste 3 porturi/grupui sunt controlate de alti 3 registri (PCMSK0, PCMSK1, PCMSK2)
  PCICR |= B00000001; // setam PCICR pentru a activa registrul PCMSK0, care controleaza pinii 8,9,10 si 11
  PCMSK0 |= B00000001; // setam registrul PCINT0 (pinul 8) sa declanseze o intrerupere
  PCMSK0 |= B00000010; // setam registrul PCINT1 (pinul 9) sa declanseze o intrerupere
  PCMSK0 |= B00000100; // setam registrul PCINT2 (pinul 10) sa declanseze o intrerupere
  PCMSK0 |= B00010000; // setam registrul PCINT3 (pinul 11) sa declanseze o intrerupere
  //Wait until the receiver is active and the throtle is set to the lower position.
  // asteptam pana cand reciver-ul este activ si throtle-ul si yaw sunt in jos (pozitia sa minima) 
  while(reciver_canal_3 < 990 || reciver_canal_3 > 1020 || reciver_canal_4 < 1400){

    reciver_canal_3 = convertire_semnal_reciver(3);// convertim semnalul efectiv al reciverului pentru throtle la standardul de 1000-2000us
    reciver_canal_4 = convertire_semnal_reciver(4);// convertim semnalul efectiv al reciverului pentru yaw la standardul de 1000-2000us
    // cat timp, esc-urile nu vor primi date sau comenzi, acestea vor bipai continuu
	// pentru ca nu vrem asta, le dam o valoare de 1000us pana cand sunt primite valorile de la reciver
	PORTD |= B11110000;// setam porturile 4, 5, 6, 7 ca high (cu tensiune electrica)
	delayMicroseconds(1000);// asteptam 1000us
	PORTD &= B00001111;// setam porturile 4, 5, 6, 7 ca low (fara tensiune electrica)
	delay(3);// asteptam 3 milisecunde pana la urmatoarea iteratie
  }
  start = 0; // setam variabila ce ne va indica startul pentru motoarele dronei, cu 0
}
// incepem programul principal, o functie care se va tot repeta 
// aici vor avea loc calculele pe care algoritmii PID ii vor face si vor comanda motoarele 
void loop(){
  // in manualul giroscopului, 65.5 LBS = 1 grad/secunda, aceasta fiind metoda prin care datele digitale sunt convertite
  // cu o regula de trei simpla, rezulta ca gyro_data/65.5 = gyro_data in grade pe secunda
  // pentru a filtra mai bine datele, acestea sunt proportionate
  giro_roll_intrare = (giro_roll_intrare * 0.7) + ((gyro_roll / 65.5) * 0.3);// valoare in grade pe secunda
  giro_pitch_intrare = (giro_pitch_intrare * 0.7) + ((gyro_pitch / 65.5) * 0.3);// valoare in grade pe secunda
  giro_yaw_intrare = (giro_yaw_intrare * 0.7) + ((gyro_yaw / 65.5) * 0.3);// valoare in grade pe secunda
  // stim ca viteza_unghiulara=distanta/timp. deci pentru a afla marimea unghiului parcurs,
  // formula va fi, distanta=viteza_unghiulara*timp
  // viteza_unghiulara este data de iesire a giroscopului iar timpul este data de frecventa (250 hz =0.004 secunde)
  //0.0000611 = 1 / 65.5 * 0.004
  unghi_pitch += gyro_pitch * 0.0000611;// calculam unghiul parcurs prin miscarea pitch, si o adunam la unghi_pitch 
  unghi_roll += gyro_roll * 0.0000611;// calculam unghiul parcurs prin miscarea roll, si o adunam la unghi_pitch 
  // in momentul in care este data o comanda yaw, drona se roteste, ceea ce implica un transfer al unghiurilor
  // pentru a putea pastra pozitia dronei corespunzatoare, in timpul zborului
  // functia care descrie cel mai bine transformarile pe care le face roll-ul sau pitch-ul este functia sinus
  // functia sinus in Arduino, poate primi doar valorile in radiani, deci va trebui sa convertim din grade in radiani
  // 0.000001066 = 0.0000611 * (3.142(PI) / 180grade) 
  unghi_pitch -= unghi_roll * sin(gyro_yaw * 0.000001066);// daca miscarea yaw transfera unghiul format prin roll in pitch
  unghi_roll += unghi_pitch * sin(gyro_yaw * 0.000001066);// daca miscarea yaw transfera unghiul format prin pitch in rolls
  // giroscopul este mai putin sensibil la vibratii si, in timp, datele acestuia intarzie
  // astfel, pentru o buna performanta, trebuie sa ne folosim de datele de la giroscop si cele de la accelerometru
  // va trebui sa calculam unghiul obtinut din vectorii dati de accelerometru
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));// calculam vectorul total al accelerometrului                                     
  unghi_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;// calculam unghiul pitch                                      
  unghi_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;// calculam unghiul roll
  // urmatoarele doua linii sunt folosite pentru calibrarea accelerometrului
  unghi_pitch_acc -= 0.0;// calibrarea accelerometrului pentru valorile pitch
  unghi_roll_acc -= 0.0;// calibrarea accelerometrului pentru valorile roll
  // corectam problemele pe care le are giroscopul cu valorile accelerometrului
  unghi_pitch = unghi_pitch * 0.9996 + unghi_pitch_acc * 0.0004;         
  unghi_roll = unghi_roll * 0.9996 + unghi_roll_acc * 0.0004;  
  // pentru a ajusta pozitia dronei, in timpul zborului, de folosim de urmatoarele doua variabile
  // 1 grad al unghiului va fi corectat cu 15 pulsatii 
  pitch_level_adjust = unghi_pitch * 15;// calculam corectiile ce vor fi aplicate pentru unghiul pitch
  roll_level_adjust = unghi_roll * 15;// calculam corectiile ce vor fi aplicate pentru unghiul roll
  // pentru a porni motoarele, vom duce throttle-ul in pozitie inferioara si vom face yaw catre stanga
  if(reciver_canal_3 < 1050 && reciver_canal_4 < 1050)
	  start = 1; // initializam variabila de start cu 1
  // cand yaw este inapoi in pozitie centrala, pornim motoarele
  if(start == 1 && reciver_canal_3 < 1050 && reciver_canal_4 > 1450){
    start = 2; // initializam variabila de start cu 2
	// in prima faza, pornim algoritmul cu valorile de initiale ale accelerometrului
    unghi_pitch = unghi_pitch_acc;// egalam ughiul pitch cu cel calculat anterior cu datele accelerometrului 
    unghi_roll = unghi_roll_acc;// egalam ughiul roll cu cel calculat anterior cu datele accelerometrului 
    unghiurile_giroscopului_setate_flag = true;// ne dorim sa facem acest lucru o singura data, deci intializam o variabila pentru asta
    // de fiecare data cand oprim si repornim motoarele, resetam variabilele ce ne vor ajuta pentru algoritmii PID
    pid_i_mem_roll = 0;// in aceasta variabila se retin valorile in procesul de integrare pentru algoritmii PID, pentru roll
    pid_ultima_eroare_d_roll = 0;// memoram ultima eroare folosita pentru procesul de derivare, pentru roll
    pid_i_mem_pitch = 0;// in aceasta variabila se retin valorile in procesul de integrare pentru algoritmii PID, pentru pitch
    pid_ultima_eroare_d_pitch = 0;// memoram ultima eroare folosita pentru procesul de derivare, pentru pitch
    pid_i_mem_yaw = 0;// in aceasta variabila se retin valorile in procesul de integrare pentru algoritmii PID, pentru yaw
    pid_ultima_eroare_d_yaw = 0;// memoram ultima eroare folosita pentru procesul de derivare, pentru yaw
  }
  // pentru a opri motoarele vom duce throttle-ul in pozitie inferioara si yaw in dreapta
  if(start == 2 && reciver_canal_3 < 1050 && reciver_canal_4 > 1950)
	  start = 0;// initializam variabila de start cu 0
  // variabila dorita este in grade pe secunda si este determinata de datele reciverului pentru miscarea roll
  // pentru a face conversia din datele provenite de la reciver in grade, se divide valoarea la 3
  pid_roll_val_ideala = 0;
  // pentru mai buna functionaliatea, se foloseste o "banda moarta"(o portiune in care valorile reciverului sunt 0)
  // astfel, tanzitia dintr-o parte in alta sa fie mult mai lina
  if(reciver_canal_1 > 1508)pid_roll_val_ideala = reciver_canal_1 - 1508;
  else if(reciver_canal_1 < 1492)pid_roll_val_ideala = reciver_canal_1 - 1492;
  pid_roll_val_ideala -= roll_level_adjust;// substragem corectiile aplicate unghiului din valoarea standard a roll-ului                  
  pid_roll_val_ideala /= 3.0;// convertim valoarea ideala in grade, prin divizarea cu 3                                         
  // la fel procedam si pentru pitch an yaw
  pid_pitch_val_ideala = 0;// pornim cu valoarea ideala egala cu 0
  if(reciver_canal_2 > 1508)pid_pitch_val_ideala = reciver_canal_2 - 1508;
  else if(reciver_canal_2 < 1492)pid_pitch_val_ideala = reciver_canal_2 - 1492;
  pid_pitch_val_ideala -= pitch_level_adjust;// substragem corectiile aplicate unghiului din valoarea standard a pitch-ului   
  pid_pitch_val_ideala /= 3.0;// convertim valoarea ideala in grade, prin divizarea cu 3   
  pid_yaw_val_ideala = 0;// pornim cu valoarea ideala egala cu 0
  if(reciver_canal_3 > 1050){ // facem o verificare, astfel incat, sa nu oprim motoarele 
    if(reciver_canal_4 > 1508)pid_yaw_val_ideala = (reciver_canal_4 - 1508)/3.0;
    else if(reciver_canal_4 < 1492)pid_yaw_val_ideala = (reciver_canal_4 - 1492)/3.0;
  }
  calculare_pid();// acum ca datele de intrare pentru algoritmul PID sunt cunoscute, calculam iesirile                   
  throttle = reciver_canal_3;// pentru a coordona esc-urile, ne folosim de semnalul throttle-ului ca semnal de baza
  if (start == 2){// verificam daca motoarele sunt pornite
  // limitam cu o valoare throtle-ul. 
    if (throttle > 1800) throttle = 1800;// in functie de rezultatele algoritmilor, se adauga sau nu diferenta de 2000 milisecunde
    esc_1 = throttle - pid_iesire_pitch + pid_iesire_roll - pid_iesire_yaw; // calculam valorile pentru esc-ul 1 (fata dreapta)
    esc_2 = throttle + pid_iesire_pitch + pid_iesire_roll + pid_iesire_yaw; // calculam valorile pentru esc-ul 2 (spate dreapta)
    esc_3 = throttle + pid_iesire_pitch - pid_iesire_roll - pid_iesire_yaw; // calculam valorile pentru esc-ul 3 (spate stanga)
    esc_4 = throttle - pid_iesire_pitch - pid_iesire_roll + pid_iesire_yaw; // calculam valorile pentru esc-ul 4 (fata stanga)
	// in functie de valorile obtinute, compensam in mod egal fiecare esc
    if (esc_1 < 1100) esc_1 = 1100;// continuam sa rotim motoarele
    if (esc_2 < 1100) esc_2 = 1100;// continuam sa rotim motoarele
    if (esc_3 < 1100) esc_3 = 1100;// continuam sa rotim motoarele
    if (esc_4 < 1100) esc_4 = 1100;// continuam sa rotim motoarele
    if(esc_1 > 2000)esc_1 = 2000;// limitam esc-urile la 2000 de milisecunde
    if(esc_2 > 2000)esc_2 = 2000;// limitam esc-urile la 2000 de milisecunde
    if(esc_3 > 2000)esc_3 = 2000;// limitam esc-urile la 2000 de milisecunde
    if(esc_4 > 2000)esc_4 = 2000;// limitam esc-urile la 2000 de milisecunde
  }
  else{// caz in care, startul nu este 2 si vrem totusi sa tinem motoarele in miscare
    esc_1 = 1000;                                                           
    esc_2 = 1000;                                                           
    esc_3 = 1000;                                                           
    esc_4 = 1000;                                                          
  }  
  // rata de improspatare este de 250 hz, deci va trebui sa schimbam datele pentru escuri la fiecare 0,004 secunde
  while(micros() - loop_timer < 4000);// asteptam pana cand cele 0.004 secunde au trecut
  loop_timer = micros();// setam timpul pentru urmatoarea iteratie
  PORTD |= B11110000;// punem sub tensiune pinii 4,5,6 si 7
  timer_canal_1 = esc_1 + loop_timer;// pentru fiecare canal, calculam timpul corespunzator esc-ului
  timer_canal_2 = esc_2 + loop_timer;// pentru fiecare canal, calculam timpul corespunzator esc-ului
  timer_canal_3 = esc_3 + loop_timer;// pentru fiecare canal, calculam timpul corespunzator esc-ului
  timer_canal_4 = esc_4 + loop_timer;// pentru fiecare canal, calculam timpul corespunzator esc-ului
  semnale_giroscop();// preluam noile date de la giroscop si convertim in grade cele de la reciver
  while(PORTD >= 16){// stam in bucla repetitiva, pana cand pinii setati anterior nu mai sunt sub tensiune
    esc_loop_timer = micros();// citim timpul real
	// cand depasim limita de timp, putem sa oprim tensiunea pentru pinii corespunzatori esc-urilor
    if(timer_canal_1 <= esc_loop_timer)PORTD &= B11101111;// eliminam tensiunea pentru pinul 4
    if(timer_canal_2 <= esc_loop_timer)PORTD &= B11011111;// eliminam tensiunea pentru pinul 5
    if(timer_canal_3 <= esc_loop_timer)PORTD &= B10111111;// eliminam tensiunea pentru pinul 6
    if(timer_canal_4 <= esc_loop_timer)PORTD &= B01111111;// eliminam tensiunea pentru pinul 7
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
  // canal 4
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
// functie care citeste de la giroscop si accelerometru date
void semnale_giroscop(){
  Wire.beginTransmission(adresa_giroscop);// incepem comunicarea cu modulul giroscop si accelerometru
  // registrul 3B retine date pentru masuratorile accelerometrului
  Wire.write(0x3B);// anuntam ca vrem sa citim, incepand cu adresa 3B (59, Accelerometer Measurements) 
  Wire.endTransmission();// oprim transmiterea de date
  Wire.requestFrom(adresa_giroscop,14);// cerem de la adresa respectiva 14 bytes
  reciver_canal_1 = convertire_semnal_reciver(1);//convertim val primita de la reciver pentru pitch la o val standard de 1000 - 2000us
  reciver_canal_2 = convertire_semnal_reciver(2);//convertim val primita de la reciver pentru roll la o val standard de 1000 - 2000us
  reciver_canal_3 = convertire_semnal_reciver(3);//convertim val primita de la reciver pentru throttle la o val standard de 1000 - 2000us
  reciver_canal_4 = convertire_semnal_reciver(4);//convertim val primita de la reciver pentru yaw la o val standard de 1000 - 2000us
  // pe masura ce citim, functia va face un auto-increment astfel incat, sa citim pana la registrul 72 (Gyroscope Measurements)
  while(Wire.available() < 14);// asteptam pana cand cei 14 bytes sunt primiti
  acc_axis[1] = Wire.read()<<8|Wire.read();// memoram byte-ul cel mai nesemnificativ si cel mai semnificativ al variabilei acc_x 
  acc_axis[2] = Wire.read()<<8|Wire.read();// memoram byte-ul cel mai nesemnificativ si cel mai semnificativ al variabilei acc_y 
  acc_axis[3] = Wire.read()<<8|Wire.read();// memoram byte-ul cel mai nesemnificativ si cel mai semnificativ al variabilei acc_z 
  temperatura = Wire.read()<<8|Wire.read();// memoram byte-ul cel mai nesemnificativ si cel mai semnificativ al variabilei temperatura 
  gyro_axis[1] = Wire.read()<<8|Wire.read();// memoram byte-ul cel mai nesemnificativ si cel mai semnificativ al valorilor unghiurilor
  gyro_axis[2] = Wire.read()<<8|Wire.read();// memoram byte-ul cel mai nesemnificativ si cel mai semnificativ al valorilor unghiurilor
  gyro_axis[3] = Wire.read()<<8|Wire.read();// memoram byte-ul cel mai nesemnificativ si cel mai semnificativ al valorilor unghiurilor
  // verificam daca a fost facuta calibrarea
  if(iterare == 2000){ // daca a fost facuta
    gyro_axis[1] -= giro_axa_dif[1];// compensam cu valorile obtinute la calibrare
    gyro_axis[2] -= giro_axa_dif[2];// compensam cu valorile obtinute la calibrare
    gyro_axis[3] -= giro_axa_dif[3];// compensam cu valorile obtinute la calibrare
  }
  // setam valorile corespunzatoare, pentru fiecare tip de date
  // pentru datele de la giroscop si accelerometru
  gyro_roll = gyro_axis[vector_date_eeprom[28] & 0b00000011]; 
  if(vector_date_eeprom[28] & 0b10000000)gyro_roll *= -1;     
  gyro_pitch = gyro_axis[vector_date_eeprom[29] & 0b00000011];
  if(vector_date_eeprom[29] & 0b10000000)gyro_pitch *= -1;       
  gyro_yaw = gyro_axis[vector_date_eeprom[30] & 0b00000011];
  if(vector_date_eeprom[30] & 0b10000000)gyro_yaw *= -1;
  acc_x = acc_axis[vector_date_eeprom[29] & 0b00000011];                           
  if(vector_date_eeprom[29] & 0b10000000)acc_x *= -1;                             
  acc_y = acc_axis[vector_date_eeprom[28] & 0b00000011];                           
  if(vector_date_eeprom[28] & 0b10000000)acc_y *= -1;                              
  acc_z = acc_axis[vector_date_eeprom[30] & 0b00000011];                           
  if(vector_date_eeprom[30] & 0b10000000)acc_z *= -1;                              
}
// functie care calculeaza valorile de iesire ai algoritmilor PID
void calculare_pid(){
  //Calulcul pentru Roll
  pid_eroare_temporara = pid_roll_val_ideala - giro_roll_intrare;// calculam eroarea
  pid_i_mem_roll += pid_i_roll * pid_eroare_temporara;// cu ajutorul parametrului de integrare, integram eroarea
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;// ne propunem o valoare maxima, pe care nu vrem sa o depasim
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  // calculam valoarea de iesire a controlerului, corespunzator formulei prezentate anterior
  pid_iesire_roll = pid_p_roll * pid_eroare_temporara + pid_i_mem_roll + pid_d_roll * (pid_eroare_temporara - pid_ultima_eroare_d_roll);
  if(pid_iesire_roll > pid_max_roll)pid_iesire_roll = pid_max_roll; // verificam daca a fost depasita valoarea maxima
  else if(pid_iesire_roll < pid_max_roll * -1)pid_iesire_roll = pid_max_roll * -1;
  // retinem ultima valoare, care ne va ajuta in calculul controlerului de derivabilitate
  pid_ultima_eroare_d_roll = pid_eroare_temporara;
  //Calulcul pentru Pitch
  pid_eroare_temporara = pid_pitch_val_ideala - giro_pitch_intrare;// calculam eroarea
  pid_i_mem_pitch += pid_i_pitch * pid_eroare_temporara;// cu ajutorul parametrului de integrare, integram eroarea
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;// ne propunem o valoare maxima, pe care nu vrem sa o depasim
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  // calculam valoarea de iesire a controlerului, corespunzator formulei prezentate anterior
  pid_iesire_pitch = pid_p_pitch * pid_eroare_temporara + pid_i_mem_pitch + pid_d_pitch * (pid_eroare_temporara - pid_ultima_eroare_d_pitch);
  if(pid_iesire_pitch > pid_max_pitch)pid_iesire_pitch = pid_max_pitch;// verificam daca a fost depasita valoarea maxima
  else if(pid_iesire_pitch < pid_max_pitch * -1)pid_iesire_pitch = pid_max_pitch * -1;
  // retinem ultima valoare, care ne va ajuta in calculul controlerului de derivabilitate
  pid_ultima_eroare_d_pitch = pid_eroare_temporara;
  //Calulcul pentru Yaw
  pid_eroare_temporara = pid_yaw_val_ideala - giro_yaw_intrare;// calculam eroarea
  pid_i_mem_yaw += pid_i_gain_yaw * pid_eroare_temporara;// cu ajutorul parametrului de integrare, integram eroarea
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;// ne propunem o valoare maxima, pe care nu vrem sa o depasim
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  // calculam valoarea de iesire a controlerului, corespunzator formulei prezentate anterior
  pid_iesire_yaw = pid_p_yaw * pid_eroare_temporara + pid_i_mem_yaw + pid_d_yaw * (pid_eroare_temporara - pid_ultima_eroare_d_yaw);
  if(pid_iesire_yaw > pid_max_yaw)pid_iesire_yaw = pid_max_yaw;// verificam daca a fost depasita valoarea maxima
  else if(pid_iesire_yaw < pid_max_yaw * -1)pid_iesire_yaw = pid_max_yaw * -1;
  // retinem ultima valoare, care ne va ajuta in calculul controlerului de derivabilitate
  pid_ultima_eroare_d_yaw = pid_eroare_temporara;
}
// aceasta functie converteste val primita de la reciver intr-o val standard de 1000-1500-2000 de microsecunde
// inseamna stabilirea unei functii afine care sa duca intervalul de valori al telecomenzii
// pentru fiecare dintre cele 4 canale, in intervalul [1000, 2000]
// pentru asta ne folosim de valorile minime, maxime si centrale, memorate in EEPROM
int convertire_semnal_reciver(byte function){
  byte canal, inversat;// declaram cateva variabile ajutatoare
  int low, center, high, semnal_efectiv;
  int diferenta;
  // fiecarui canal ii corespunde o anumita functie, care defineste o anumita miscare
  // spre exemplu, pentru canalul 3, este atribuita miscare throttle-ului, adica 1
  // acesta memorare este facuta, pentru a se pastra o ordine
  canal = vector_date_eeprom[function + 23] & 0b00000111;// verificam din EEPROM, care miscare corespunde cu functia
  if(vector_date_eeprom[function + 23] & 0b10000000)inversat = 1; // caz in care canalele sunt inversate
  else inversat = 0;
  semnal_efectiv = receiver_input[canal];// pentru ca stim canalul corespunzator, vom sti ce data de intrare a reciverului sa alegem
  // pentru fiecare canal, luam cea mai mica valoarea, cea mai mare si cea centrala
  low = (vector_date_eeprom[canal * 2 + 15] << 8) | vector_date_eeprom[canal * 2 + 14];//memoram valoarea minima pentru canalul specific
  center = (vector_date_eeprom[canal * 2 - 1] << 8) | vector_date_eeprom[canal * 2 - 2];//memoram valoarea centrala pentru canalul specific
  high = (vector_date_eeprom[canal * 2 + 7] << 8) | vector_date_eeprom[canal * 2 + 6];//memoram valoarea maxima ce pentru canalul specific
  // pentru calculare, in functie de semnal, alegem functia corespunzatoare
  // f(semnal_reciver) = 1500 + (semnal_reciver - semnal_central)/(semnal_central-semnal_minim)*500
  if(semnal_efectiv < center){// cand semnalul efectiv este mai mic decat cel central
    if(semnal_efectiv < low)semnal_efectiv = low;// limitam valoarea la cea care a fost detectata in timpul calibrarii
    diferenta = ((long)(center - semnal_efectiv) * (long)500) / (center - low);       
    if(inversat == 1)return 1500 + diferenta;//daca canalul este inversat
    else return 1500 - diferenta;//daca canalul nu este inversat
  }// f(semnal_reciver) = 1500 + (semnal_reciver - semnal_central)/(semnal_maxim-semnal_central)*500
  else if(semnal_efectiv > center){// cand semnalul efectiv este mai mare decat cel central
    if(semnal_efectiv > high)semnal_efectiv = high;// limitam valoarea la cea care a fost detectata in timpul calibrarii
    diferenta = ((long)(semnal_efectiv - center) * (long)500) / (high - center);    
    if(inversat == 1)return 1500 - diferenta;//daca canalul este inversat
    else return 1500 + diferenta;//daca canalul nu este inversat
  }
  else return 1500;// daca valoare corespunde cu cea centrala
}
// functie folosita pentru a configura modulul giroscop si accelerometru
void configurare_giroscop(){
  // incepem sa configuram modulul giroscop si accelerometru
  // pentru a seta anumite caracteristici ale datelor pe care le vom primi
  Wire.beginTransmission(adresa_giroscop);// incepem comunicarea cu giroscopul si accelerometru
  //in prima faza, giroscopul vine mod activat, sleep mode
  //acesta va trebui schimbat, pentru a putea porni giroscopul
  Wire.write(0x6B);// registrul 107 (PWR_MGMT_1) permite configurarea modulului de alimentare si sursa ceasului
  Wire.write(0x00);// schimbam valoarea registrului PWR_MGMT_1 la 0 pentru a porni giroscopul
  Wire.endTransmission();// oprim transmiterea de date si trimitem registrii
  Wire.beginTransmission(adresa_giroscop);// incepem comunicarea cu giroscopul si accelerometru 
  Wire.write(0x1B);// anuntam ca vrem sa scriem in registrul GYRO_CONFIG
  Wire.write(0x08);// setam bitii registrului cu 00001000 (1 pentru bitul FS_SEL=Full scale range)(500dps)- degree per second
  // astfel, selectam gama completa de valori ale giroscopului
  Wire.endTransmission();// oprim transmiterea de date
  Wire.beginTransmission(adresa_giroscop);// incepem comunicarea cu giroscopul si accelerometru
  //0x1C sau registrul ACCEL_CONFIG este folosit pentru a seta intervalul pe care datele accelerometrului sa le aiba(+/- 2g/16g)
  Wire.write(0x1C);// vrem sa scrim la adresa 28 ( registrul ACCEL_CONFIG)
  Wire.write(0x10);// setam bitii registrului cu 00010000, pentru a stabili valoarea intervalului de date (+/- 8g full scale range)
  Wire.endTransmission();// oprim transmiterea de date
  Wire.beginTransmission(adresa_giroscop);// incepem comunicarea cu giroscopul si accelerometru 
  // prin natura sa, giroscopul este sensibil la vibratii, astfel, pentru performante mai bune putem configura un registru
  // registrul 1A sau 26 este folosit pentru configurare
  Wire.write(0x1A);// vrem sa scrim la adresa 26 (registrul CONFIG)
  Wire.write(0x03);// setam bitii registrului cu 00000011 DLPF (Digital Low Pass Filter) la aproximativ 43Hz
  Wire.endTransmission();// oprim transmiterea de date     
}
// functie care copiaza datele din eeprom intr-un vector
void copiere_eeprom(){
  for(start = 0; start <= 33; start++)
	  vector_date_eeprom[start] = EEPROM.read(start);
  start = 0;// setam valoarea de start inapoi la 0
  adresa_giroscop = vector_date_eeprom[32];// memoram adresa modulului giroscop si accelerometru
}
void calibrare_giroscop(){
  for (iterare = 0; iterare < 2000 ; iterare ++){ //luam 2000 de date de la giroscop.
	semnale_giroscop();// se citesc date de la giroscop si accelerometru
	giro_axa_med[1] += gyro_axis[1];// adaugam valoarea pentru roll corespunzatoare giroscopului 
	giro_axa_val[1][iterare] = gyro_axis[1];
	giro_axa_med[2] += gyro_axis[2];//adaugam valoarea pentru pitch corespunzatoare giroscopului 
	giro_axa_val[2][iterare] = gyro_axis[2];
	giro_axa_med[3] += gyro_axis[3];//adaugam valoarea pentru yaw corespunzatoare giroscopului 
	giro_axa_val[3][iterare] = gyro_axis[3];
	// cat timp, esc-urile nu vor primi date sau comenzi, acestea vor bipai continuu
	// pentru ca nu vrem asta, le dam o valoare de 1000us si apoi le oprim
	PORTD |= B11110000;// setam porturile 4, 5, 6, 7 ca high (cu tensiune electrica)
	delayMicroseconds(1000);// asteptam 1000us
	PORTD &= B00001111;// setam porturile 4, 5, 6, 7 ca low (fara tensiune electrica)
	delay(3);// asteptam 3 milisecunde pana la urmatoarea iteratie
	}
  // dupa ce avem 2000 de masuratori, putem sa facem o medie cu acestea pentru a gasi "eroarea" giroscopului
  giro_axa_med[1] /= 2000;// impartim la 2000 valorile pentru roll
  giro_axa_med[2] /= 2000;// impartim la 2000 valorile pentru pitch
  giro_axa_med[3] /= 2000;// impartim la 2000 valorile pentru yaw
  for (int i = 0; i < 2000 ; i ++){ // adunam distanta drinte fiecare masuratore si media anterioara
    giro_axa_dif[1] += (giro_axa_val[1][i] - giro_axa_med[1]) * (giro_axa_val[1][i] - giro_axa_med[1]);
	giro_axa_dif[2] += (giro_axa_val[2][i] - giro_axa_med[2]) * (giro_axa_val[2][i] - giro_axa_med[2]);
	giro_axa_dif[3] += (giro_axa_val[3][i] - giro_axa_med[3]) * (giro_axa_val[3][i] - giro_axa_med[3]);
  }
  //calculam abaterea standard de la media obtinuta anterior
  giro_axa_dif[1] /= 2000;
  giro_axa_dif[2] /= 2000;
  giro_axa_dif[3] /= 2000;
  giro_axa_dif[1] = sqrt(giro_axa_dif[1]);
  giro_axa_dif[2] = sqrt(giro_axa_dif[2]);
  giro_axa_dif[3] = sqrt(giro_axa_dif[3]);

}
