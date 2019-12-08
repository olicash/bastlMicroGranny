// op qr st   uv wx yz
unsigned char incomingByte,note;
boolean ignore, comandOff,slave=false,cc; //,midiNoteOn  noteOn,
unsigned char state=0;
int ll;

uint16_t clockCounter;
#define MIDI_CHANNEL 1023
#define POLYPHONY NUMBER_OF_VOICES
unsigned char notesInBuffer=ZERO;
boolean thereIsNoteToPlay;
unsigned char sound, activeSound;
unsigned char inputChannel;
#define BUFFER_SIZE 16
unsigned char midiBuffer[BUFFER_SIZE];
#define SYSEX_BUFFER_SIZE 125
unsigned char sysexCtr=0;
unsigned char sysexBuffer[SYSEX_BUFFER_SIZE];
#define EMPTY 255
unsigned char midiVelocity;
unsigned char fromBuffer;
unsigned char instantLoop;
boolean sustain;
boolean legato;
//unsigned char sideChannel=0;
//unsigned char sideNote=1;
//unsigned char sideDecay;
//int pitchBendNow;   
int sampleRateNow;
unsigned char setting;
int attackInterval, releaseInterval;
//long legatoPosition; 
//const uint16_t
PROGMEM const uint16_t noteSampleRateTable[49]={/*0-C*/
  2772,2929,3103,3281,3500,3679,3910,4146,4392,4660,4924,5231,5528,5863,6221,6579,6960,7355,7784,8278,8786,9333,9847,10420,11023,/*11*/ 11662,12402,13119,13898,14706,15606,16491,17550,18555,19677,20857, /*0*/22050,23420,24807,26197,27815,29480,29480,29480,29480,29480,29480,29480,/*48-C*/29480};


void shiftBufferLeft(unsigned char from){
  for(uint8_t i=from;i<notesInBuffer;i++){
    midiBuffer[i]=midiBuffer[i+1]; 
  }
}

void shiftBufferRight(){
  for(uint8_t i=notesInBuffer;i>ZERO;i--){
    midiBuffer[i]=midiBuffer[i-1]; 
  }
}

boolean isThereNoteToPlay(){
  return thereIsNoteToPlay;

}
unsigned char noteToPlay(){
  thereIsNoteToPlay=false;
  return midiBuffer[fromBuffer];
}

void putNoteIn(unsigned char note){
  if(note<6) hw.freezeAllKnobs();
  if(notesInBuffer==BUFFER_SIZE-1) removeNote(midiBuffer[BUFFER_SIZE-1]);
  removeNote(note); // check if the note is already in the buffer if yes remove it
  if(notesInBuffer<BUFFER_SIZE){ //put the note to the first position
    if(notesInBuffer>ZERO){ 
      shiftBufferRight();
    }
    midiBuffer[ZERO]=note; // put the last note to the first place 
    notesInBuffer++;
    thereIsNoteToPlay=true;
    fromBuffer=ZERO;
  }

  if(thereIsNoteToPlay) {
    if(legato && note>=23 && note<66 && notesInBuffer>1) sound=note,sampleRateNow=(pgm_read_word_near(noteSampleRateTable+sound-23)),wave.setSampleRate(sampleRateNow);
    else playSound(midiBuffer[ZERO]);
  }
  //  hw.freezAllKnobs();
}

void clearBuffer(){
  for(uint8_t i=ZERO;i<BUFFER_SIZE;i++) midiBuffer[i]=EMPTY;
  notesInBuffer=0;
}

boolean removeNote(unsigned char note){
  if(notesInBuffer>ZERO){ 
    unsigned char takenOut;
    boolean takeOut=ZERO;

    for(uint8_t i=ZERO;i<notesInBuffer;i++){
      if(midiBuffer[i]==note) takeOut=true, takenOut=i;
    } 

    if(takeOut){
      shiftBufferLeft(takenOut);
      notesInBuffer--;
      for(uint8_t i=notesInBuffer;i<BUFFER_SIZE;i++) midiBuffer[i]=EMPTY;
      return true;
    }
    else return false;

  }
}

unsigned char putNoteOut(unsigned char note){
  if(note<6) hw.freezeAllKnobs();

  if(removeNote(note)){

    if(notesInBuffer>0){
      if(midiBuffer[ZERO]!=sound) {
        if(legato && midiBuffer[ZERO]>=23 && midiBuffer[ZERO]<66) sound=midiBuffer[ZERO],sampleRateNow=(pgm_read_word_near(noteSampleRateTable+sound-23)),wave.setSampleRate(sampleRateNow);
        else playSound(midiBuffer[ZERO]),instantLoop=0;
      } //legatoPosition=wave.getCurPosition(),
    }
    else if(!sustain) stopEnvelope(),instantLoop=0;
    return midiBuffer[ZERO];

  }
}



void initMidi(){
  clearBuffer();
  //Serial.begin(9600);
  readMidiChannel();
  //Serial.end();
  Serial.begin(MIDI_BAUD);

}
/*
#define SIDE_CHANNEL 1022
#define SIDE_NOTE 1021
#define SIDE_DECAY 1020
*/
unsigned char TOLERANCE=3;
//#define TOLERANCE 3
unsigned char controler, CCvalue;
#define TOL_EE 1019

void readMidiChannel(){

//  sideChannel=EEPROM.read(SIDE_CHANNEL);

//  sideNote=EEPROM.read(SIDE_NOTE);
  //sideDecay=EEPROM.read(SIDE_DECAY);
  inputChannel=EEPROM.read(MIDI_CHANNEL);
  if(inputChannel>16) EEPROM.write(MIDI_CHANNEL,0), inputChannel=0;//,EEPROM.write(TOL_EE,0);
  

  //TOLERANCE=EEPROM.read(TOL_EE);
  //  if(TOLERANCE>10) EEPROM.write(TOL_EE,0);
 
   hw.update();
  if(hw.buttonState(PAGE)) EEPROM.write(TOL_EE,0);
  if(hw.buttonState(REC)) EEPROM.write(TOL_EE,4);
  if(hw.buttonState(HOLD)) EEPROM.write(TOL_EE,9);
  TOLERANCE = 3 +  EEPROM.read(TOL_EE);
  if(TOLERANCE!=12 && TOLERANCE!=7 && TOLERANCE!=3) TOLERANCE=3;
  //Serial.print(TOLERANCE);
 
  for(uint8_t i=0;i<6;i++){
    if(hw.buttonState(bigButton[i])){
      /*
      if(hw.buttonState(UP)) sideChannel=i+6*hw.buttonState(FN), EEPROM.write(SIDE_CHANNEL,sideChannel);//, showValue(sideChannel+1);//, hw.displayChar('C',1); 
      else if(hw.buttonState(DOWN)) sideNote=i+60*hw.buttonState(FN), EEPROM.write(SIDE_NOTE,sideNote);//, showValue(sideNote), hw.displayChar('N',1); 
      else 
      */
      inputChannel=i+6*hw.buttonState(FN),EEPROM.write(MIDI_CHANNEL,inputChannel);
    }
  }
  showValue(inputChannel+1);
  hw.displayChar('C',0);
  hw.displayChar('H',1); 
  //hw.setDot(3,true);
  hw.setDot(VERSION,true);


  if(wave.isPlaying()){
    while(!wave.isPaused()) hw.update();
  }
  stopSound();
  //noDots();
}

long lastClockPosition, clockLength;
//unsigned char pByte1,pByte2;
//boolean pb;
//boolean side;
int bytesAvailable;

void readMidi(){
  //channel=map(analogRead(4),0,1024,0,16);
  while(Serial.available() > 0){
    bytesAvailable=Serial.available();
    if (bytesAvailable <= 0) return;
    if(bytesAvailable>=64) Serial.flush(); // If the buffer is full -> Don't Panic! Call the Vogons to destroy it.
    else handleByte(Serial.read());
  }
}

boolean noteOnStatus=false, noteOffStatus=false, ccStatus=false, sysexStatus=false, firstByte=true;

unsigned char number, value;
unsigned char channel=0;

#define SYSEX_START_BYTE 0xF0
#define SYSEX_END_BYTE 0xF7
#define SDS_DUMP_HEADER 0x01
#define SDS_DATA_PACKET 0x02

int sampleLength, sampleCtr;
unsigned char bitDepth, lastPacketNumber, checkSum;
bool writeSysex=false;
char fileName[7]="AA.WAV";

void handleByte(unsigned char incomingByte){

  if (sysexStatus){
    if (incomingByte==SYSEX_END_BYTE || (sysexCtr>=125 && incomingByte!=SYSEX_END_BYTE)){
      if (incomingByte==SYSEX_END_BYTE && sysexBuffer[1]==0x7E){
        if (sysexBuffer[3]==SDS_DUMP_HEADER && checkSDSHeaderValid()){
          sampleLength=(sysexBuffer[10]&0x7F)|((sysexBuffer[11]&0x7F)<<7)|((sysexBuffer[12]&0x7F)<<14);
          writeSysex=startSysexWrite();
        }
        else if (sysexBuffer[3]==SDS_DATA_PACKET && writeSysex && sysexBuffer[4]!=lastPacketNumber){
          lastPacketNumber=(lastPacketNumber+1)&0x7F;
          for (int i=0;i<125;i++) checkSum^=(sysexBuffer[i]&0x7F);
          if (sysexBuffer[4]!=lastPacketNumber || sysexBuffer[124]!=checkSum) abortSysexWrite();
          else writeSysexBuffer();
        }
      }
      sysexStatus=false,sysexCtr=0;
    }
    else sysexBuffer[sysexCtr++]=incomingByte;
  }
  else if(handleRealTime(incomingByte));
  else{
    if(incomingByte>127) recognizeStatus(incomingByte);

    else if(firstByte){
      firstByte=false;
      if(ccStatus) number=incomingByte;
      else if(noteOnStatus || noteOffStatus) number=incomingByte;
      else firstByte=true;
    }
    else{
      value=incomingByte;
      firstByte=true; 
      if(ccStatus) handleCC(number,value, channel);
      else if(noteOnStatus) handleNote(number,value, channel);
      else if(noteOffStatus) handleNote(number,0, channel);
    }
  } 
}

#define NOTE_ON_BYTE 0x09
#define NOTE_OFF_BYTE 0x08
#define CC_BYTE 0x0B

void recognizeStatus(unsigned char incomingByte){
  if (incomingByte==SYSEX_START_BYTE){
    noteOnStatus=false, noteOffStatus=false, ccStatus=false, sysexStatus=true;
    sysexCtr=0;
    return;
  }
  
  channel=0;
  for(uint8_t i=0;i<4;i++) bitWrite(channel,i,bitRead(incomingByte,i));

  incomingByte=incomingByte>>4;
  if(incomingByte==NOTE_ON_BYTE) noteOnStatus=true, noteOffStatus=false, ccStatus=false, sysexStatus=false;
  else if(incomingByte==NOTE_OFF_BYTE) noteOnStatus=false, noteOffStatus=true, ccStatus=false, sysexStatus=false;
  else if(incomingByte==CC_BYTE) noteOnStatus=false, noteOffStatus=false, ccStatus=true, sysexStatus=false;
  else noteOnStatus=false, noteOffStatus=false, ccStatus=false, sysexStatus=false;
  firstByte=true;
  //channel=
}

void handleNote(unsigned char _number,unsigned char _value,unsigned char _channel){
 if(_number>6) _number--;
  if(_channel==inputChannel){
    if(_value==0) putNoteOut(_number);
    else  midiVelocity=126,putNoteIn(_number);
  } 

  //if(_channel==sideChannel && _number==sideNote && _value!=0) startEnvelope(midiVelocity,attackInterval);
}

void handleCC(unsigned char _number,unsigned char _value,unsigned char _channel){
  if(_channel==inputChannel){
    proceedCC(_number,_value);
  }
}

boolean handleRealTime(unsigned char _incomingByte){
  if((_incomingByte>=0xF8) && (_incomingByte<=0xFF)){
    if(_incomingByte==0xF8){ //clock
      clockCounter++;
      slave=true;
    }
    else if(_incomingByte==0xFA ){ //start
      clockCounter=0;
      slave=true;
      // setSetting(activeSound);
    }
    else if(_incomingByte==0xFC){ //stop
      clockCounter=0;
      slave=true;
    } 
    return true;
  }
  else return false;
}


#define SUSTAIN_PEDAL_BYTE 64
#define PRESET_BY_CC_BYTE 0
#define BANK_BY_CC_BYTE 1
#define RANDOMIZE_BYTE 127

#define CONTROL_CHANGE_BITS 7
#define CONTROL_CHANGE_OFFSET 102
#define CONTROL_CHANGE_OFFSET_2 110

void proceedCC(unsigned char _number,unsigned char _value){

  if(_number==1) setVar(activeSound,CRUSH,_value), hw.unfreezeAllKnobs(),renderTweaking(0),hw.freezeAllKnobs(); //modwheel
  //if(_number==123) clearBuffer(),sound=0, stopEnvelope(),instantLoop=0; // all notes off
  else if(_number==SUSTAIN_PEDAL_BYTE){ 
    sustain=_value>>6;
    if(!sustain && notesInBuffer==0) stopEnvelope(),instantLoop=0;  
  }

  else if(_number==PRESET_BY_CC_BYTE) loadPreset(currentBank,myMap(_value,128,NUMBER_OF_PRESETS));
  //if(_number==BANK_BY_CC_BYTE) loadPreset(myMap(_value,128,NUMBER_OF_BANKS),currentPreset);

  // else if(_number==RANDOMIZE_BYTE) randomize(activeSound);

  else if(_number>=CONTROL_CHANGE_OFFSET && _number<CONTROL_CHANGE_OFFSET_2){
    _number=_number-CONTROL_CHANGE_OFFSET;
    setVar(activeSound,_number,scale(_value,CONTROL_CHANGE_BITS,variableDepth[_number]));  
    hw.unfreezeAllKnobs();
    renderTweaking(_number/VARIABLES_PER_PAGE);
    hw.freezeAllKnobs();
  }
  //if(_number==123) clearBuffer();

}

/*
void proceedPB(unsigned char _byte1,unsigned char _byte2){
 int pitchBendNow=word(_byte1,_byte2)-8192; 
 wave.setSampleRate(sampleRateNow+pitchBendNow);
 }
 */

typedef struct Wav_header {
  char riff_header[4]={'R','I','F','F'};
  int wav_size; // Size of the wav portion of the file, which follows the first 8 bytes. File size - 8
  char wave_header[4]={'W','A','V','E'};
  char fmt_header[4]={'f','m','t',' '};
  int fmt_chunk_size=16;
  short audio_format=1;
  short num_channels=1;
  int sample_rate=22050;
  int byte_rate; // Number of bytes per second. sample_rate * num_channels * Bytes Per Sample
  short sample_alignment; // num_channels * Bytes Per Sample
  short bit_depth; // Number of bits per sample
  char data_header[4]={'d','a','t','a'};
  int data_bytes; // Number of bytes in data. Number of samples * num_channels * sample byte size
} Wav_header;

bool checkSDSHeaderValid(){
  if (sysexBuffer[6]!=8 || sysexBuffer[6]!=16) return false;
  bitDepth=sysexBuffer[6];
  int sp=(sysexBuffer[7]&0x7F)|((sysexBuffer[8]&0x7F)<<7)|((sysexBuffer[9]&0x7F)<<14);
  if (sp!=45351) return false;  // 1000000000/22050
  return true;
}

bool startSysexWrite(){
  stopSound();

  int fileNum=(sysexBuffer[4]&0x7F)|((sysexBuffer[5]&0x7F)<<7);
  int firstChar=fileNum/36;
  if (firstChar>25) return false;
  fileName[0]=firstChar+65;
  int secondChar=fileNum%36;
  if (secondChar>25) fileName[1]=48+secondChar-26;
  else fileName[1]=secondChar+65;
  
  if (file.open(&root, fileName, O_READ)){
    file.close();
    if (!SdFile::remove(&root, fileName)) return false;
  }
  if (!file.createContiguous(&root, fileName, MAX_FILE_SIZE)) return false;
  if (!file.open(&root, fileName, O_WRITE)) return false;

  Wav_header wav_header;
  wav_header.sample_alignment=wav_header.bit_depth/8;
  wav_header.data_bytes=sampleLength*wav_header.sample_alignment;
  wav_header.wav_size=sizeof(wav_header) + wav_header.data_bytes - 8;
  wav_header.byte_rate=wav_header.sample_rate*wav_header.sample_alignment;
  if (!file.write((const void*)&wav_header,sizeof(wav_header)))
  {
    file.close();
    SdFile::remove(&root, fileName);
    return false;
  }
  lastPacketNumber=checkSum=0;
  sampleCtr=0;
  return true;
}

void writeSysexBuffer(){
  if (bitDepth==16){
    unsigned short sample;
    for (int i=0;i<125 && sampleCtr<sampleLength;i+=3){
      sample = ((unsigned short)sysexBuffer[i] << 9) | ((unsigned short)sysexBuffer[i+1] << 2) | (sysexBuffer[i+2] >> 5);
      if (!file.write((const void*)&sample,sizeof(unsigned short))) abortSysexWrite();
      sampleCtr++;
    }
  }
  else{  // bitDepth==8 
    unsigned char sample;
    for (int i=0;i<125 && sampleCtr<sampleLength;i+=2){
      sample = ((unsigned char)sysexBuffer[i] << 1) | (sysexBuffer[i+1] >> 6);
      if (!file.write((const void*)&sample,sizeof(unsigned char))) abortSysexWrite();
      sampleCtr++;
    }
  }
  if (sampleCtr>=sampleLength){
    file.close();
    showForWhile("sysx");
  }
}

void abortSysexWrite(){
  file.close();
  SdFile::remove(&root, fileName);
  writeSysex=false;
}
