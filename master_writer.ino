#include <iarduino_I2C_Software.h>

SoftTwoWire sWire1(3,4); 
SoftTwoWire sWire2(5,6); 
SoftTwoWire sWire3(7,8); 

int8_t found = 0;

void blinks( int times ){
  for( int i = 0; i < times; i++ ){
     digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
     delay(150);                      // wait for a second
     digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
     delay(150);                      // wait for a second
  }
  delay(2000 - ( times * 300 ));                      // wait for a second
}

void writeByte( SoftTwoWire &w, uint8_t addr, uint8_t reg, uint8_t byte ){
  w.beginTransmission(addr); // transmit to device
  w.write(reg);              // sends one byte
  w.write(byte);              // sends one byte
  w.endTransmission();    // stop transmitting
}

void MA12070P_configure( SoftTwoWire &w ){
   writeByte(w, 0x20, 0x00, 0x5D); // 0010 0000 Manual Power Mode Profile 2.
   writeByte(w, 0x20, 0x1D, 0x01); // 0000 0010 Power Mode Profile 1.
   writeByte(w, 0x20, 0x0A, 0x80); // 1000 0000 Enables soft-clipping. 
   writeByte(w, 0x20, 0x35, 0x00); // Source: i2s

   //writeByte(w, 0x20, 0x35, 0xA8); // 1010 1000 быстрая атака, быстрый спад, использовать аудиопроцессор, i2s
   writeByte(w, 0x20, 0x36, 0x41); // 0100 0001 Use the limiter.
   writeByte(w, 0x20, 0x47, 0x1A); // 0001 1010 Use the limiter. 26дБ
   writeByte(w, 0x20, 0x48, 0x1A); // 0001 1010 Use the limiter. 26дБ
   writeByte(w, 0x20, 0x49, 0x1A); // 0001 1010 Use the limiter. 26дБ
   writeByte(w, 0x20, 0x4A, 0x1A); // 0001 1010 Use the limiter. 26дБ

   Serial.println("End configure MA12070P.");
}

void setup() {
   pinMode(9, OUTPUT);
   pinMode(10, OUTPUT);
   pinMode(11, OUTPUT);
   digitalWrite(9, 0);
   digitalWrite(10, 0);
   digitalWrite(11, 0);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  
  Serial.println("Find devices...");
  blinks( 3 );

  sWire1.begin();
  for (uint8_t i = 0, j = 0; i <= 127; i++)
  {
    sWire1.beginTransmission(i);
    uint8_t result = sWire1.endTransmission();
    if( result == 0 ){
      Serial.print("Device with address 0x");
      Serial.print(i, HEX);
      Serial.println(" found on 3,4 pin");
      found++;
      MA12070P_configure( sWire1 );
    }
    if( i == 127 && found == 0 ){
       i = 0;
    }
  }

  blinks( found );

  sWire2.begin();
  for (uint8_t i = 0; i <= 127; i++)
  {
    sWire2.beginTransmission(i);
    uint8_t result = sWire2.endTransmission();
    if( result == 0 ){
      Serial.print("Device with address 0x");
      Serial.print(i, HEX);
      Serial.println(" found on 5,6 pin");
      found++;
      MA12070P_configure( sWire2 );
    }
    if( i == 127 && found == 1 )
       i = 0;
  }

  blinks( found );

  sWire3.begin();
  for (uint8_t i = 0; i <= 127; i++)
  {
    sWire3.beginTransmission(i);
    uint8_t result = sWire3.endTransmission();
    if( result == 0 ){
      Serial.print("Device with address 0x");
      Serial.print(i, HEX);
      Serial.println(" found on 7,8 pin");
      found++;
      MA12070P_configure( sWire3 );
    }
    if( i == 127 && found == 2 )
       i = 0;
  }

  blinks( found );

  Serial.print("Found ");
  Serial.print(found);
  Serial.println(" devices.");
   
   digitalWrite(9, 1);
   digitalWrite(10, 1);
   digitalWrite(11, 1);
}

void MA12070P_read( SoftTwoWire &w ){
   // read registers MA12070P
   int adr = 0x20;
   for (uint8_t reg = 0; reg <= 127; reg++) {
     w.beginTransmission(adr);   // Инициируем передачу данных по шине I2C к устройству с адресом 0x20. При этом сама передача не начнётся.
     w.write(reg);               // Помещаем в буфер для передачи один байт (адрес первого читаемого регистра).
     w.endTransmission(false);   // Выполняем инициированную ранее передачу данных, без установки STOP.
     w.requestFrom(adr, 1);    // Читаем (запрашиваем) 1 байт данных от устройства с адресом 0x20.

//   Выводим прочитанные данные:     //
     while( w.available() ){     // Если в буфере остались данные...
       Serial.print("Reg 0x");
       Serial.print(reg, HEX);
       Serial.print(" : 0x");
       Serial.println(sWire1.read(), HEX); // Выводим очередной прочитанный байт.
     } 

    }
}

void MA12070P_watch( SoftTwoWire &w, int blink ){
  int adr = 0x20;
  uint8_t reg = 0x7C;
  w.beginTransmission(adr);   // Инициируем передачу данных по шине I2C к устройству с адресом 0x20. При этом сама передача не начнётся.
  w.write(reg);               // Помещаем в буфер для передачи один байт (адрес первого читаемого регистра).
  w.endTransmission(false);   // Выполняем инициированную ранее передачу данных, без установки STOP.
  w.requestFrom(adr, 1);    // Читаем (запрашиваем) 1 байт данных от устройства с адресом 0x20.

//   Выводим прочитанные данные:     //
  while( w.available() ){     // Если в буфере остались данные...
    //Serial.print("Reg 0x");
    //Serial.print(reg, HEX);
    //Serial.print(" : 0x");
    //Serial.println(w.read(), HEX); // Выводим очередной прочитанный байт.
    uint8_t value = w.read();
    if( value == 0 ){
       //blinks( blink );
     digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
     delay(25);                      // wait for a second
     digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
     delay(25);                      // wait for a second
    } 
    else { // ошибка усилителя -> гасим всех.
      digitalWrite(9, 0);
      digitalWrite(10, 0);
      digitalWrite(11, 0);

      blinks( blink );
      blinks( blink );
      for( uint8_t i = 0, bit = 1; i < 8; i++, bit <<= 1 ){
         if( value & bit )
            blinks( i );
      }
    }
  } 

}

void loop() {

 MA12070P_watch( sWire1, 1 );
 MA12070P_watch( sWire2, 2 );
 MA12070P_watch( sWire3, 3 );

}
