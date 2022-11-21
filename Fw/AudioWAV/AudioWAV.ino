/***********************************************************************************
 * MIT License
 * 
 * Application Copyright (c) C Gerrish (BigG/Gerrikoio) - relates to this code
 * 
 * Copyright (c) 2021 Robin Grosset - relates to PWM audio
 * https://github.com/rgrosset/pico-pwm-audio
 * 
 * Source on how to read header info from WAV audio file:
 * https://stackoverflow.com/questions/13660777/c-reading-the-data-part-of-a-wav-file
 * 

You will notice in the code a "while (!Serial) {;;}". 
This is basically to pause playing the audio until the serial monitor is opened. 
This is not required and is simply there for demo purposes.

Another important point to make about the code is that I am buffering the whole file into RAM. 
This limits the size of the audio file to about 10 seconds for 8 bit sampling rate at 11kHz.

https://audio.online-convert.com/es/convertir-a-wav
 
This was simply a short cut to get me started. 
Please note that there are no size checks in the code, so it's best to amend if larger audio files want to be tested.
**********************************************************************************/

#include <stdio.h>
#include "stdlib.h"   // stdlib 
#include "hardware/irq.h"  // interrupts
#include "hardware/pwm.h"  // pwm 
#include "hardware/sync.h" // wait for interrupt 

#include "hardware/pll.h"
#include "hardware/clocks.h"

#include "PluggableUSBMSD.h"
#include "FlashIAPBlockDevice.h"

// Note that file is written to the actual root directory as created when USBMSD is mounted
// File is not stored within a folder called root.
// File is only found when you remove usb and replace again
const char *fname = "/root/chona.wav";

static FlashIAPBlockDevice bd(XIP_BASE + 0x100000, 0x100000);
USBMSD MassStorage(&bd);

static FILE *f = nullptr;

// Audio PIN is to match some of the design guide shields. 
#define AUDIO_PIN 28  // you can change this to whatever you like

#define AMP_EN 5 //Enable Ampli

uint32_t wav_position = 0;

uint32_t ChunkSize = 0;                 // this is the actual sound data size
uint8_t* WAV_DATA;

typedef struct  WAV_HEADER
{
    /* RIFF Chunk Descriptor */
    uint8_t         RIFF[4];        // RIFF Header Magic header
    uint32_t        ChunkSize;      // RIFF Chunk Size
    uint8_t         WAVE[4];        // WAVE Header
    /* "fmt" sub-chunk */
    uint8_t         fmt[4];         // FMT header
    uint32_t        Subchunk1Size;  // Size of the fmt chunk
    uint16_t        AudioFormat;    // Audio format 1=PCM,6=mulaw,7=alaw,     257=IBM Mu-Law, 258=IBM A-Law, 259=ADPCM
    uint16_t        NumOfChan;      // Number of channels 1=Mono 2=Sterio
    uint32_t        SamplesPerSec;  // Sampling Frequency in Hz
    uint32_t        bytesPerSec;    // bytes per second
    uint16_t        blockAlign;     // 2=16-bit mono, 4=16-bit stereo
    uint16_t        bitsPerSample;  // Number of bits per sample
    /* "data" sub-chunk */
    uint8_t         Subchunk2ID[4]; // "data"  string
    uint32_t        Subchunk2Size;  // Sampled data length
} wav_hdr;


/***********************************************************************************
 * Inline functions to enable overclocking
 * 
************************************************************************************/ 
void set_sys_clock_pll(uint32_t vco_freq, uint post_div1, uint post_div2) {
  if (!running_on_fpga()) {
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    pll_init(pll_sys, 1, vco_freq, post_div1, post_div2);
    uint32_t freq = vco_freq / (post_div1 * post_div2);

    // Configure clocks
    // CLK_REF = XOSC (12MHz) / 1 = 12MHz
    clock_configure(clk_ref,
                    CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
                    0,  // No aux mux
                    12 * MHZ,
                    12 * MHZ);

    // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    freq, freq);

    clock_configure(clk_peri,
                    0,  // Only AUX mux on ADC
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);
  }
}
bool check_sys_clock_khz(uint32_t freq_khz, uint *vco_out, uint *postdiv1_out, uint *postdiv_out) {
  uint crystal_freq_khz = clock_get_hz(clk_ref) / 1000;
  for (uint fbdiv = 320; fbdiv >= 16; fbdiv--) {
    uint vco = fbdiv * crystal_freq_khz;
    if (vco < 400000 || vco > 1600000) continue;
    for (uint postdiv1 = 7; postdiv1 >= 1; postdiv1--) {
      for (uint postdiv2 = postdiv1; postdiv2 >= 1; postdiv2--) {
        uint out = vco / (postdiv1 * postdiv2);
        if (out == freq_khz && !(vco % (postdiv1 * postdiv2))) {
          *vco_out = vco * 1000;
          *postdiv1_out = postdiv1;
          *postdiv_out = postdiv2;
          return true;
        }
      }
    }
  }
  return false;
}
static inline bool set_sys_clock_khz(uint32_t freq_khz, bool required) {
  uint vco, postdiv1, postdiv2;
  if (check_sys_clock_khz(freq_khz, &vco, &postdiv1, &postdiv2)) {
    set_sys_clock_pll(vco, postdiv1, postdiv2);
    return true;
  } else if (required) {
    panic("System clock of %u kHz cannot be exactly achieved", freq_khz);
  }
  return false;
}

/********************************************************************/

void USBMSD::begin()
{
  int err = getFileSystem().mount(&bd);
  if (err) {
   //String FN = getFileSystem().getName();
   Serial.println(" filesystem mount failed. Try to reformat device...");
    err = getFileSystem().reformat(&bd);
  }
  if (err) {
    Serial.println("Error: Unable to format/mount the device.");
    while(1);
  }
}

mbed::FATFileSystem &USBMSD::getFileSystem()
{
  static mbed::FATFileSystem fs("root");
  return fs;
}

/*
 * PWM Interrupt Handler which outputs PWM level and advances the 
 * current sample. 
 * 
 * We repeat the same value for 8 cycles this means sample rate etc
 * adjust by factor of 8   (this is what bitshifting <<3 is doing)
 * 
 */
 
void pwm_interrupt_handler() {
    pwm_clear_irq(pwm_gpio_to_slice_num(AUDIO_PIN));    
    if (wav_position < (ChunkSize<<3) - 1) { 
        // set pwm level 
        // allow the pwm value to repeat for 8 cycles this is >>3 
        pwm_set_gpio_level(AUDIO_PIN, WAV_DATA[wav_position>>3]);  
        wav_position++;
    } else {
        // reset to start
        wav_position = 0;
    }
}

void readContents() {
    wav_hdr wavHeader;
    int headerSize = sizeof(wav_hdr);
    
    f = fopen(fname, "r");
    if (f != nullptr) {
      size_t bytesRead = fread(&wavHeader, 1, headerSize, f);
      Serial.print("Header Read "); Serial.print(bytesRead); Serial.println(" bytes.");
      if (bytesRead > 0) {
        //Read the data
        uint16_t bytesPerSample = wavHeader.bitsPerSample / 8;      //Number     of bytes per sample
        uint64_t numSamples = wavHeader.ChunkSize / bytesPerSample; //How many samples are in the wav file?
        //static const uint16_t BUFFER_SIZE = 4096;
        
        Serial.println();
        Serial.print("RIFF header                :"); Serial.print((char)wavHeader.RIFF[0]); Serial.print((char)wavHeader.RIFF[1]); Serial.print((char)wavHeader.RIFF[2]); Serial.println((char)wavHeader.RIFF[3]);
        Serial.print("Chunk Size                 :"); Serial.print(wavHeader.ChunkSize); Serial.print(" TOTAL: "); Serial.print(wavHeader.ChunkSize+8);  Serial.print(" DATA: "); Serial.println(wavHeader.ChunkSize-(36 + wavHeader.Subchunk2Size)-8);
        Serial.print("WAVE header                :"); Serial.print((char)wavHeader.WAVE[0]); Serial.print((char)wavHeader.WAVE[1]); Serial.print((char)wavHeader.WAVE[2]); Serial.println((char)wavHeader.WAVE[3]);
        Serial.print("Subchunk1 ID (fmt)         :"); Serial.print((char)wavHeader.fmt[0]); Serial.print((char)wavHeader.fmt[1]); Serial.print((char)wavHeader.fmt[2]); Serial.println((char)wavHeader.fmt[3]);
        Serial.print("Subchunk1 size             :"); Serial.print(wavHeader.Subchunk1Size); Serial.println(wavHeader.Subchunk1Size==16?" PCM":"");

        // Display the sampling Rate from the header
        Serial.print("Audio Format               :"); Serial.println(wavHeader.AudioFormat==1?"PCM":(wavHeader.AudioFormat==6?"mulaw":(wavHeader.AudioFormat==7?"alaw":(wavHeader.AudioFormat==257?"IBM Mu-Law":(wavHeader.AudioFormat==258?"IBM A-Law":"ADPCM")))));
        Serial.print("Number of channels         :"); Serial.println(wavHeader.NumOfChan==1?"Mono":(wavHeader.NumOfChan==2?"Mono":"Other"));
        Serial.print("Sampling Rate              :"); Serial.println(wavHeader.SamplesPerSec);
        Serial.print("Number of bytes per second :"); Serial.println(wavHeader.bytesPerSec);
        Serial.print("Block align                :"); Serial.print(wavHeader.blockAlign); Serial.print(" validate: "); Serial.println(bytesPerSample * wavHeader.NumOfChan);
        Serial.print("Number of bits per sample  :"); Serial.println(wavHeader.bitsPerSample);
        Serial.print("Data length                :"); Serial.println(wavHeader.Subchunk2Size);
        // Audio format 1=PCM,6=mulaw,7=alaw, 257=IBM Mu-Law, 258=IBM A-Law, 259=ADPCM

        Serial.print("Data string (Subchunk2 ID) :"); Serial.print((char)wavHeader.Subchunk2ID[0]); Serial.print((char)wavHeader.Subchunk2ID[1]); Serial.print((char)wavHeader.Subchunk2ID[2]); Serial.println((char)wavHeader.Subchunk2ID[3]);
        Serial.print("Subchunk2 size             :"); Serial.print(wavHeader.Subchunk2Size); Serial.print(" validate: "); Serial.println(numSamples * wavHeader.NumOfChan * bytesPerSample);

        if (memcmp((char*)wavHeader.Subchunk2ID,"LIST",4) == 0) {
          Serial.println("List chunk (of a RIFF file):");
          
          uint8_t ListType[wavHeader.Subchunk2Size];        // RIFF Header Magic header
          
          bytesRead = fread(&ListType, 1, wavHeader.Subchunk2Size, f);
          
          if ( bytesRead > 0 ) {
            Serial.print(" --- List type ID :"); Serial.print((char)ListType[0]); Serial.print((char)ListType[1]); Serial.print((char)ListType[2]); Serial.println((char)ListType[3]);
            if (memcmp((char*)ListType,"INFO",4) == 0) {
              // INFO tag
              Serial.print(" --- --- INFO1 type ID :"); Serial.print((char)ListType[4]); Serial.print((char)ListType[5]); Serial.print((char)ListType[6]); Serial.println((char)ListType[7]);
              uint8_t sizeTxt = (ListType[11] << 24) | (ListType[10] << 16) | (ListType[9] << 8) | ListType[8];
              Serial.print(" --- --- SizeD :"); Serial.print(sizeTxt); Serial.print(" Validate: "); Serial.println(wavHeader.Subchunk2Size - sizeTxt);
            }
            for (uint8_t x = 12; x < wavHeader.Subchunk2Size; x++) {
              if (ListType[x] >= 32 && ListType[x] <= 126) {
                Serial.print((char)ListType[x]);
              }
              else if (ListType[x] > 0) {
                Serial.print("0x"); Serial.print(ListType[x],HEX);
              }
              Serial.print(" ");
            }
            Serial.println("");

            // Checking for data
            bytesRead = fread(&ListType, 1, 4, f);
            if (bytesRead > 0) {
              Serial.print("Data string (Subchunk3 ID) :"); 
              for (uint8_t x = 0; x < 4; x++) {
                if (ListType[x] >= 32 && ListType[x] <= 126) {
                  Serial.print((char)ListType[x]);
                }
                else if (ListType[x] > 0) {
                  Serial.print("0x"); Serial.print(ListType[x],HEX);
                }               
              }
              Serial.println("");
            }

            bytesRead = fread(&ChunkSize, 1, 4, f);
            if (bytesRead > 0) {
              Serial.print("Subchunk3 size             :");
              Serial.print(ChunkSize); Serial.print(" validate: "); Serial.println(8+numSamples * wavHeader.NumOfChan * bytesPerSample);
            }
          }
        }
        else {
          ChunkSize = wavHeader.Subchunk2Size;
        }

        WAV_DATA = new uint8_t[ChunkSize];

        //int8_t* buffer = new int8_t[BUFFER_SIZE];
        bytesRead = fread(WAV_DATA, sizeof WAV_DATA[0], ChunkSize / (sizeof WAV_DATA[0]), f);
        if (bytesRead)
        {
          Serial.println("Sound File Data Read ");
          
          int fileSize = 0;
          fseek(f, 0, SEEK_END);
          fileSize = ftell(f);
          fseek(f, 0, SEEK_SET);
          Serial.print("File size is: "); Serial.print(fileSize); Serial.println(" bytes.");
          
          fclose(f);
      
          gpio_set_function(AUDIO_PIN, GPIO_FUNC_PWM);
      
          int audio_pin_slice = pwm_gpio_to_slice_num(AUDIO_PIN);
      
          // Setup PWM interrupt to fire when PWM cycle is complete
          pwm_clear_irq(audio_pin_slice);
          pwm_set_irq_enabled(audio_pin_slice, true);
          // set the handle function above
          irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_interrupt_handler); 
          irq_set_enabled(PWM_IRQ_WRAP, true);
      
          // Setup PWM for audio output
          pwm_config config = pwm_get_default_config();
          /* Base clock 176,000,000 Hz divide by wrap 250 then the clock divider further divides
           * to set the interrupt rate. 
           * 
           * 11 KHz is fine for speech. Phone lines generally sample at 8 KHz
           * 
           * 
           * So clkdiv should be as follows for given sample rate
           *  8.0f for 11 KHz
           *  4.0f for 22 KHz
           *  2.0f for 44 KHz etc
           */
          pwm_config_set_clkdiv(&config, 8.0f); 
          pwm_config_set_wrap(&config, 250); 
          pwm_init(audio_pin_slice, &config, true);
      
          pwm_set_gpio_level(AUDIO_PIN, 0);
              
        }
        //delete [] buffer;
        //buffer = nullptr;
      }
      
    }
    else {
      Serial.println("File not found");
    }  
}

void setup() {

  set_sys_clock_khz(176000, true); 
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(AMP_EN,OUTPUT);
  digitalWrite(AMP_EN,LOW); //Disable Ampli
  MassStorage.begin();
  // If you do not want to wait for Serial Monitor to load then use a long delay here.
  // Found a delay helps if you want to capture the initial serial output.
  while(!Serial) {;;}
  //delay(1000);
  Serial.println("\r\nMassStorage mounted");
  Serial.println("");
  Serial.flush();
  digitalWrite(AMP_EN,HIGH); // Enable ampli
  //writeContents();
  readContents();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  __wfi(); // Wait for Interrupt
}
