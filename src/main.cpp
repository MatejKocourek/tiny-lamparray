#define DISABLEMILLIS
#include <Arduino.h>
#include <usbdrv.h>
#include <avr/sleep.h>
#include <avr/power.h>
//#include <HidSensorSpec.h>
#include <util/delay.h>
#include <SK6812_io.cpp>

// HID Report Descriptor
PROGMEM const uchar usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
    0x05, 0x59,                      // UsagePage(Lighting And Illumination[0x0059])
    0x09, 0x01,                      // UsageId(LampArray[0x0001])
    0xA1, 0x01,                      // Collection(Application)
    0x85, 0x01,                      //     ReportId(1)
    0x09, 0x02,                      //     UsageId(LampArrayAttributesReport[0x0002])
    0xA1, 0x02,                      //     Collection(Logical)
    0x09, 0x03,                      //         UsageId(LampCount[0x0003])
    0x15, 0x00,                      //         LogicalMinimum(0)
    0x27, 0xFF, 0xFF, 0x00, 0x00,    //         LogicalMaximum(65,535)
    0x95, 0x01,                      //         ReportCount(1)
    0x75, 0x10,                      //         ReportSize(16)
    0xB1, 0x03,                      //         Feature(Constant, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, NonVolatile, BitField)
    0x09, 0x04,                      //         UsageId(BoundingBoxWidthInMicrometers[0x0004])
    0x09, 0x05,                      //         UsageId(BoundingBoxHeightInMicrometers[0x0005])
    0x09, 0x06,                      //         UsageId(BoundingBoxDepthInMicrometers[0x0006])
    0x09, 0x07,                      //         UsageId(LampArrayKind[0x0007])
    0x09, 0x08,                      //         UsageId(MinUpdateIntervalInMicroseconds[0x0008])
    0x27, 0xFF, 0xFF, 0xFF, 0x7F,    //         LogicalMaximum(2,147,483,647)
    0x95, 0x05,                      //         ReportCount(5)
    0x75, 0x20,                      //         ReportSize(32)
    0xB1, 0x03,                      //         Feature(Constant, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, NonVolatile, BitField)
    0xC0,                            //     EndCollection()
    0x85, 0x02,                      //     ReportId(2)
    0x09, 0x20,                      //     UsageId(LampAttributesRequestReport[0x0020])
    0xA1, 0x02,                      //     Collection(Logical)
    0x09, 0x21,                      //         UsageId(LampId[0x0021])
    0x27, 0xFF, 0xFF, 0x00, 0x00,    //         LogicalMaximum(65,535)
    0x95, 0x01,                      //         ReportCount(1)
    0x75, 0x10,                      //         ReportSize(16)
    0xB1, 0x02,                      //         Feature(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, NonVolatile, BitField)
    0xC0,                            //     EndCollection()
    0x85, 0x03,                      //     ReportId(3)
    0x09, 0x22,                      //     UsageId(LampAttributesResponseReport[0x0022])
    0xA1, 0x02,                      //     Collection(Logical)
    0x09, 0x21,                      //         UsageId(LampId[0x0021])
    0xB1, 0x02,                      //         Feature(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, NonVolatile, BitField)
    0x09, 0x23,                      //         UsageId(PositionXInMicrometers[0x0023])
    0x09, 0x24,                      //         UsageId(PositionYInMicrometers[0x0024])
    0x09, 0x25,                      //         UsageId(PositionZInMicrometers[0x0025])
    0x09, 0x27,                      //         UsageId(UpdateLatencyInMicroseconds[0x0027])
    0x09, 0x26,                      //         UsageId(LampPurposes[0x0026])
    0x27, 0xFF, 0xFF, 0xFF, 0x7F,    //         LogicalMaximum(2,147,483,647)
    0x95, 0x05,                      //         ReportCount(5)
    0x75, 0x20,                      //         ReportSize(32)
    0xB1, 0x02,                      //         Feature(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, NonVolatile, BitField)
    0x09, 0x28,                      //         UsageId(RedLevelCount[0x0028])
    0x09, 0x29,                      //         UsageId(GreenLevelCount[0x0029])
    0x09, 0x2A,                      //         UsageId(BlueLevelCount[0x002A])
    0x09, 0x2B,                      //         UsageId(IntensityLevelCount[0x002B])
    0x09, 0x2C,                      //         UsageId(IsProgrammable[0x002C])
    0x09, 0x2D,                      //         UsageId(InputBinding[0x002D])
    0x26, 0xFF, 0x00,                //         LogicalMaximum(255)
    0x95, 0x06,                      //         ReportCount(6)
    0x75, 0x08,                      //         ReportSize(8)
    0xB1, 0x02,                      //         Feature(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, NonVolatile, BitField)
    0xC0,                            //     EndCollection()
    0x85, 0x04,                      //     ReportId(4)
    0x09, 0x50,                      //     UsageId(LampMultiUpdateReport[0x0050])
    0xA1, 0x02,                      //     Collection(Logical)
    0x09, 0x03,                      //         UsageId(LampCount[0x0003])
    0x25, 0x08,                      //         LogicalMaximum(8)
    0x95, 0x01,                      //         ReportCount(1)
    0xB1, 0x02,                      //         Feature(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, NonVolatile, BitField)
    0x09, 0x55,                      //         UsageId(LampUpdateFlags[0x0055])
    0x25, 0x01,                      //         LogicalMaximum(1)
    0xB1, 0x02,                      //         Feature(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, NonVolatile, BitField)
    0x09, 0x21,                      //         UsageId(LampId[0x0021])
    0x27, 0xFF, 0xFF, 0x00, 0x00,    //         LogicalMaximum(65,535)
    0x95, 0x08,                      //         ReportCount(8)
    0x75, 0x10,                      //         ReportSize(16)
    0xB1, 0x02,                      //         Feature(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, NonVolatile, BitField)
    0x09, 0x51,                      //         UsageId(RedUpdateChannel[0x0051])
    0x09, 0x52,                      //         UsageId(GreenUpdateChannel[0x0052])
    0x09, 0x53,                      //         UsageId(BlueUpdateChannel[0x0053])
    0x09, 0x54,                      //         UsageId(IntensityUpdateChannel[0x0054])
    0x09, 0x51,                      //         UsageId(RedUpdateChannel[0x0051])
    0x09, 0x52,                      //         UsageId(GreenUpdateChannel[0x0052])
    0x09, 0x53,                      //         UsageId(BlueUpdateChannel[0x0053])
    0x09, 0x54,                      //         UsageId(IntensityUpdateChannel[0x0054])
    0x09, 0x51,                      //         UsageId(RedUpdateChannel[0x0051])
    0x09, 0x52,                      //         UsageId(GreenUpdateChannel[0x0052])
    0x09, 0x53,                      //         UsageId(BlueUpdateChannel[0x0053])
    0x09, 0x54,                      //         UsageId(IntensityUpdateChannel[0x0054])
    0x09, 0x51,                      //         UsageId(RedUpdateChannel[0x0051])
    0x09, 0x52,                      //         UsageId(GreenUpdateChannel[0x0052])
    0x09, 0x53,                      //         UsageId(BlueUpdateChannel[0x0053])
    0x09, 0x54,                      //         UsageId(IntensityUpdateChannel[0x0054])
    0x09, 0x51,                      //         UsageId(RedUpdateChannel[0x0051])
    0x09, 0x52,                      //         UsageId(GreenUpdateChannel[0x0052])
    0x09, 0x53,                      //         UsageId(BlueUpdateChannel[0x0053])
    0x09, 0x54,                      //         UsageId(IntensityUpdateChannel[0x0054])
    0x09, 0x51,                      //         UsageId(RedUpdateChannel[0x0051])
    0x09, 0x52,                      //         UsageId(GreenUpdateChannel[0x0052])
    0x09, 0x53,                      //         UsageId(BlueUpdateChannel[0x0053])
    0x09, 0x54,                      //         UsageId(IntensityUpdateChannel[0x0054])
    0x09, 0x51,                      //         UsageId(RedUpdateChannel[0x0051])
    0x09, 0x52,                      //         UsageId(GreenUpdateChannel[0x0052])
    0x09, 0x53,                      //         UsageId(BlueUpdateChannel[0x0053])
    0x09, 0x54,                      //         UsageId(IntensityUpdateChannel[0x0054])
    0x09, 0x51,                      //         UsageId(RedUpdateChannel[0x0051])
    0x09, 0x52,                      //         UsageId(GreenUpdateChannel[0x0052])
    0x09, 0x53,                      //         UsageId(BlueUpdateChannel[0x0053])
    0x09, 0x54,                      //         UsageId(IntensityUpdateChannel[0x0054])
    0x26, 0xFF, 0x00,                //         LogicalMaximum(255)
    0x95, 0x20,                      //         ReportCount(32)
    0x75, 0x08,                      //         ReportSize(8)
    0xB1, 0x02,                      //         Feature(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, NonVolatile, BitField)
    0xC0,                            //     EndCollection()
    0x85, 0x05,                      //     ReportId(5)
    0x09, 0x60,                      //     UsageId(LampRangeUpdateReport[0x0060])
    0xA1, 0x02,                      //     Collection(Logical)
    0x09, 0x55,                      //         UsageId(LampUpdateFlags[0x0055])
    0x25, 0x01,                      //         LogicalMaximum(1)
    0x95, 0x01,                      //         ReportCount(1)
    0xB1, 0x02,                      //         Feature(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, NonVolatile, BitField)
    0x09, 0x61,                      //         UsageId(LampIdStart[0x0061])
    0x09, 0x62,                      //         UsageId(LampIdEnd[0x0062])
    0x27, 0xFF, 0xFF, 0x00, 0x00,    //         LogicalMaximum(65,535)
    0x95, 0x02,                      //         ReportCount(2)
    0x75, 0x10,                      //         ReportSize(16)
    0xB1, 0x02,                      //         Feature(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, NonVolatile, BitField)
    0x09, 0x51,                      //         UsageId(RedUpdateChannel[0x0051])
    0x09, 0x52,                      //         UsageId(GreenUpdateChannel[0x0052])
    0x09, 0x53,                      //         UsageId(BlueUpdateChannel[0x0053])
    0x09, 0x54,                      //         UsageId(IntensityUpdateChannel[0x0054])
    0x26, 0xFF, 0x00,                //         LogicalMaximum(255)
    0x95, 0x04,                      //         ReportCount(4)
    0x75, 0x08,                      //         ReportSize(8)
    0xB1, 0x02,                      //         Feature(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, NonVolatile, BitField)
    0xC0,                            //     EndCollection()
    0x85, 0x06,                      //     ReportId(6)
    0x09, 0x70,                      //     UsageId(LampArrayControlReport[0x0070])
    0xA1, 0x02,                      //     Collection(Logical)
    0x09, 0x71,                      //         UsageId(AutonomousMode[0x0071])
    0x25, 0x01,                      //         LogicalMaximum(1)
    0x95, 0x01,                      //         ReportCount(1)
    0xB1, 0x02,                      //         Feature(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, NonVolatile, BitField)
    0xC0,                            //     EndCollection()
    0xC0                             // EndCollection()
    };


#define LAMP_NOT_PROGRAMMABLE 0x00
#define LAMP_IS_PROGRAMMABLE 0x01

#define LAMP_UPDATE_FLAG_UPDATE_COMPLETE 1

#define LED_OUT() pinMode(0, OUTPUT)
#define LED_OFF() digitalWrite(0, 0)
#define LED_ON() digitalWrite(0, 1)
#define LED_TOGGLE() digitalWrite(0, !digitalRead(LED_BUILTIN))

enum LampPurposeKind
{
    LampPurposeControl = 1,
    LampPurposeAccent = 2,
    LampPurposeBranding = 4,
    LampPurposeStatus = 8,
    LampPurposeIllumination = 16,
    LampPurposePresentation = 32,
};

enum LampArrayKind
{
    LampArrayKindKeyboard = 1,
    LampArrayKindMouse = 2,
    LampArrayKindGameController = 3,
    LampArrayKindPeripheral = 4,
    LampArrayKindScene = 5,
    LampArrayKindNotification = 6,
    LampArrayKindChassis = 7,
    LampArrayKindWearable = 8,
    LampArrayKindFurniture = 9,
    LampArrayKindArt = 10,
};

struct __attribute__ ((__packed__)) LampArrayColor
{
    uint8_t RedChannel;
    uint8_t GreenChannel;
    uint8_t BlueChannel;
    uint8_t IntensityChannel;
};

struct __attribute__ ((__packed__)) LampAttributes
{
    uint16_t LampId;
    uint32_t PositionXInMillimeters;
    uint32_t PositionYInMillimeters;
    uint32_t PositionZInMillimeters;
    uint32_t UpdateLatencyInMicroseconds;
    uint32_t LampPurposes;
    uint8_t RedLevelCount;
    uint8_t GreenLevelCount;
    uint8_t BlueLevelCount;
    uint8_t IntensityLevelCount;
    uint8_t IsProgrammable;
    uint8_t LampKey;
};

#define LAMP_ARRAY_ATTRIBUTES_REPORT_ID 1
struct __attribute__ ((__packed__)) LampArrayAttributesReport
{
    uint8_t  ReportId;
    uint16_t LampCount;
    uint32_t BoundingBoxWidthInMillimeters;
    uint32_t BoundingBoxHeightInMillimeters;
    uint32_t BoundingBoxDepthInMillimeters;
    uint32_t LampArrayKind;
    uint32_t MinUpdateIntervalInMicroseconds;
};

#define LAMP_ATTRIBUTES_REQUEST_REPORT_ID 2
struct __attribute__ ((__packed__)) LampAttributesRequestReport
{
    uint8_t ReportId;
    uint16_t LampId;
};

#define LAMP_ATTRIBUTES_RESPONSE_REPORT_ID 3
struct __attribute__ ((__packed__)) LampAttributesResponseReport
{
    uint8_t ReportId;
    LampAttributes Attributes;
} lampAttributeReport;

#define LAMP_MULTI_UPDATE_REPORT_ID 4
#define LAMP_MULTI_UPDATE_LAMP_COUNT 8
struct __attribute__ ((__packed__)) LampMultiUpdateReport
{
    uint8_t ReportId;
    uint8_t LampCount;
    uint8_t LampUpdateFlags;
    uint16_t LampIds[LAMP_MULTI_UPDATE_LAMP_COUNT];
    LampArrayColor UpdateColors[LAMP_MULTI_UPDATE_LAMP_COUNT];
};


#define LAMP_RANGE_UPDATE_REPORT_ID 5
struct __attribute__ ((__packed__)) LampRangeUpdateReport
{
    uint8_t ReportId;
    uint8_t LampUpdateFlags;
    uint16_t LampIdStart;
    uint16_t LampIdEnd;
    LampArrayColor UpdateColor;
};

#define LAMP_ARRAY_CONTROL_REPORT_ID 6
struct __attribute__ ((__packed__)) LampArrayControlReport
{
    uint8_t ReportId;
    uint8_t AutonomousMode;
};


constexpr uint8_t pinStrip = 8u;
constexpr uint16_t lampsPerMeter = 60;
constexpr uint16_t m_lampCount = 37u;

constexpr float m_lampSpacing = 1000.0/lampsPerMeter * 1000ul;
constexpr uint32_t m_boundingBoxWidthInMicrometers = m_lampCount * m_lampSpacing;
constexpr uint32_t m_boundingBoxHeightInMicrometers = 0ul;
constexpr uint32_t m_boundingBoxDepthInMicrometers = 0ul;
constexpr uint32_t m_minUpdateInternalInMicroseconds = 33333ul;

PROGMEM const LampArrayAttributesReport lampArrayAttributesReport{
  .ReportId = LAMP_ARRAY_ATTRIBUTES_REPORT_ID,
  .LampCount = m_lampCount,
  .BoundingBoxWidthInMillimeters = m_boundingBoxWidthInMicrometers,
  .BoundingBoxHeightInMillimeters = m_boundingBoxHeightInMicrometers,
  .BoundingBoxDepthInMillimeters = m_boundingBoxDepthInMicrometers,
  .LampArrayKind = LampArrayKind::LampArrayKindPeripheral,
  .MinUpdateIntervalInMicroseconds = m_minUpdateInternalInMicroseconds
};

struct GRBW{
  uint8_t G;
  uint8_t R;
  uint8_t B;
  uint8_t W;

  inline bool operator==(const GRBW& rhs)
  {
    return G==rhs.G && R==rhs.R && B == rhs.B && W == rhs.W;
  }
  inline bool operator!=(const GRBW& rhs)
  {
    return G!=rhs.G || R!=rhs.R || B != rhs.B || W != rhs.W;
  }
};



template <typename T>
constexpr T smallest(T x, T y, T z)
{
  return x < y ? (x < z ? x : z) : (y < z ? y : z);
}

const uint8_t PROGMEM gamma8[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

uint8_t gammaCorrect(uint8_t color)
{
  return pgm_read_byte(&gamma8[color]);
}

constexpr GRBW RGBtoRGBW(uint8_t r, uint8_t g, uint8_t b)
{
  //constexpr uint8_t r = 255;
  //constexpr uint8_t g = 212;
  //constexpr uint8_t b = 177;
  uint8_t w_out = smallest(r, g, b);


  constexpr uint8_t white_rgb_r = 255;
  constexpr uint8_t white_rgb_g = 255;
  constexpr uint8_t white_rgb_b = 255;

  constexpr uint8_t white_white_r = 255;
  constexpr uint8_t white_white_g = 206;
  constexpr uint8_t white_white_b = 176;

  constexpr float r_a = 1.0 * white_white_r/white_rgb_r;
  constexpr float g_a = 1.0 * white_white_g/white_rgb_g;
  constexpr float b_a = 1.0 * white_white_b/white_rgb_b;


  uint8_t r_out = r-(uint8_t)(w_out*r_a);
  uint8_t g_out = g-(uint8_t)(w_out*g_a);
  uint8_t b_out = b-(uint8_t)(w_out*b_a);

  return GRBW{
    .G = g_out,
    .R = r_out,
    .B = b_out,
    .W = w_out
  };
}

GRBW gammaCorrect(GRBW in)
{
  return GRBW{
    .G = gammaCorrect(in.G),
    .R = gammaCorrect(in.R),
    .B = gammaCorrect(in.B),
    .W = gammaCorrect(in.W)
  };
}


struct MyLed{
  static constexpr void exportGeneralAttributes(LampAttributes& attributes)
  {
    attributes.UpdateLatencyInMicroseconds = m_minUpdateInternalInMicroseconds;//2500;//400Hz
    attributes.LampPurposes = LampPurposeKind::LampPurposeAccent;
    attributes.RedLevelCount = 255;
    attributes.GreenLevelCount = 255;
    attributes.BlueLevelCount = 255;
    attributes.IntensityLevelCount = 1;
    attributes.IsProgrammable = 1;
    attributes.LampKey = 0;

    attributes.PositionYInMillimeters = 0;
    attributes.PositionZInMillimeters = 0;
  }

  static constexpr void exportIndividualAttributes(uint16_t lampId, LampAttributes& attributes)
  {
    attributes.LampId = lampId;
    attributes.PositionXInMillimeters = lampId*(m_boundingBoxWidthInMicrometers/m_lampCount);
  }

  bool set(uint8_t R, uint8_t G, uint8_t B, uint8_t W)
  {
    bool changed = color.R!=R || color.G!=G || color.B!=B || color.W!= W;
    color.R = R;
    color.G = G;
    color.B = B;
    color.W = W; //TODO

    return changed;
  }
  bool set(uint8_t r, uint8_t g, uint8_t b)
  {
    auto newColor = gammaCorrect(RGBtoRGBW(r,g,b));
    bool res = color!=newColor;
    color = newColor;
    return res;
  }
  
  bool set(const LampArrayColor& lampArrayColor)
  {
    return set(lampArrayColor.RedChannel,lampArrayColor.GreenChannel,lampArrayColor.BlueChannel);
  }

  GRBW color;
};

typedef MyLed LED;

struct LEDStrip
{
  LEDStrip()
  {
    _pin_mask = digitalPinToBitMask(pinStrip);
    _port = portOutputRegister(digitalPinToPort(pinStrip));
    _port_reg = portModeRegister(digitalPinToPort(pinStrip));
  }

  int16_t updateLedsToIndex = -1;
  LED leds[m_lampCount] = {0};

  uint8_t _pin_mask;
  volatile uint8_t *_port;
  volatile uint8_t *_port_reg;

  void setColor(uint16_t index, const LampArrayColor &lampArrayColor)
  {
    if (leds[index].set(lampArrayColor))
      updateLedsToIndex = max(updateLedsToIndex, (int16_t)index);
  }

  void updateStrip()
  {
    *_port_reg |= _pin_mask;

    //updateLedsToIndex = 35;
    sendarray_mask((uint8_t *)leds, sizeof(*leds) * (updateLedsToIndex + 1), _pin_mask, (uint8_t *)_port, (uint8_t *)_port_reg);

    updateLedsToIndex = -1;
  }

} ledStrip;

uint16_t m_lastLampIdRequested = 0;
bool m_isAutonomousMode = true;

void setup() {
  ADCSRA &= ~(1<<ADEN);
  power_adc_disable ();
  power_spi_disable();
  power_twi_disable();
  //LED_OUT();
  //LED_OFF();

  lampAttributeReport.ReportId = LAMP_ATTRIBUTES_RESPONSE_REPORT_ID;
  LED::exportGeneralAttributes(lampAttributeReport.Attributes);

  noInterrupts();
  usbInit();
  usbDeviceDisconnect(); /* enforce re-enumeration */

  byte timeoutCounter = 0;
  do
  { /* fake USB disconnect for > 250 ms */
    _delay_ms(1);
  } while (--timeoutCounter);
  usbDeviceConnect();
  interrupts();
}


template<typename T>
usbMsgLen_t SendFeatureReport(const T& data) noexcept
{
  usbMsgPtr = (uchar *)&data;
  usbMsgFlags &= ~USB_FLG_MSGPTR_IS_ROM;
  return sizeof(data);
}

usbMsgLen_t SendLampArrayAttributesReport() noexcept
{
  usbMsgPtr = (uchar *)&lampArrayAttributesReport;
  usbMsgFlags |= USB_FLG_MSGPTR_IS_ROM;
  return sizeof(lampArrayAttributesReport);
}

usbMsgLen_t SendLampAttributesReport() noexcept
{
  //LampAttributesResponseReport lampAttributeReport;
  //lampAttributeReport.ReportId = LAMP_ATTRIBUTES_RESPONSE_REPORT_ID;
  //LED::exportGeneralAttributes(lampAttributeReport.Attributes);

  LED::exportIndividualAttributes(m_lastLampIdRequested, lampAttributeReport.Attributes);

  m_lastLampIdRequested = (m_lastLampIdRequested+1)%m_lampCount;

  return SendFeatureReport(lampAttributeReport);
}

bool updateLeds = false;


void UpdateRequestLampFromLampAttributesRequestReport(const LampAttributesRequestReport &report) noexcept
{
  // Per HID spec, if not within bounds, always set LampId to 0.
  if (report.LampId < m_lampCount) [[likely]]
  {
    m_lastLampIdRequested = report.LampId;
  }
  else
  {
    m_lastLampIdRequested = 0;
  }
}

void UpdateLampStateCacheFromMultiUpdateReport(const LampMultiUpdateReport &report) noexcept
{
  //[[assume(report.LampCount <= LAMP_MULTI_UPDATE_LAMP_COUNT)]]
  for (uint8_t i = 0; i < report.LampCount; i++)
  {
    // Ignore update if not within bounds.
    if (report.LampIds[i] < m_lampCount) [[likely]]
    {
      ledStrip.setColor(report.LampIds[i], report.UpdateColors[i]);
    }
  }

  // Don't want the consumer to update before the Host has completed the batch of updates.
  if (report.LampUpdateFlags & LAMP_UPDATE_FLAG_UPDATE_COMPLETE)
  {
    updateLeds = true;
  }
}

void UpdateLampStateCacheFromRangeUpdateReport(const LampRangeUpdateReport &report) noexcept
{
  // Ignore update if not within bounds.
  if (report.LampIdStart >= 0 && report.LampIdStart < m_lampCount &&
      report.LampIdEnd >= 0 && report.LampIdEnd < m_lampCount &&
      report.LampIdStart <= report.LampIdEnd) [[likely]]
  {
    for (uint8_t i = report.LampIdStart; i <= report.LampIdEnd; i++)
    {
      ledStrip.setColor(i, report.UpdateColor);
    }

    // Don't want the consumer to update before the Host has completed the batch of updates.
    if (report.LampUpdateFlags & LAMP_UPDATE_FLAG_UPDATE_COMPLETE)
    {
      updateLeds = true;
    }
  }
}

void ProcessControlReport(const LampArrayControlReport& report) noexcept
{
  m_isAutonomousMode = !!report.AutonomousMode;
}

void loop()
{
  usbPoll();

  if (updateLeds)
  {
    updateLeds = false;

    uint8_t timeoutCounter = 0;
    do
    {
      usbPoll();
    } while (--timeoutCounter);
    //_delay_ms(1);
    ledStrip.updateStrip();
  }
}

union ReceivedData
{
  uint8_t ReportId;
  LampAttributesRequestReport attributesRequest;
  LampMultiUpdateReport multiUpdate;
  LampRangeUpdateReport rangeUpdate;
  LampArrayControlReport control;
} receivedData;

struct hidRequest{
    uint8_t     bmRequestType;
    uint8_t     bRequest;
    uint8_t     ReportID;
    uint8_t     ReportType;
    usbWord_t   wIndex;
    usbWord_t   wLength;
};



uint16_t bytesToRead;
uchar * writeBuf;

#ifdef __cplusplus
extern "C"
{
#endif
  usbMsgLen_t usbFunctionSetup(uchar data[8])
  {
    hidRequest *rq = (hidRequest *)data;
    if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) /* class request type */
    {
      switch (rq->bRequest)
      {
      case USBRQ_HID_GET_REPORT:
      {
        switch (rq->ReportID)
        {
        case LAMP_ARRAY_ATTRIBUTES_REPORT_ID:
          return SendLampArrayAttributesReport();
        case LAMP_ATTRIBUTES_RESPONSE_REPORT_ID:
          return SendLampAttributesReport();
        }
      }
      break;
      case USBRQ_HID_SET_REPORT:
      {
        switch (rq->ReportID)
        {
        case LAMP_ATTRIBUTES_REQUEST_REPORT_ID:
        case LAMP_MULTI_UPDATE_REPORT_ID:
        case LAMP_RANGE_UPDATE_REPORT_ID:
        case LAMP_ARRAY_CONTROL_REPORT_ID:
          //receivedType = rq->ReportID;
          bytesToRead = rq->wLength.word;
          writeBuf = (unsigned char*) &receivedData;
          return USB_NO_MSG; /* Use usbFunctionWrite() to get data from host */
        break;
        default: [[unlikely]]
          break;
        }
      }
      break;
      default:
        break;
      }
    }
    return 0; /* default for not implemented requests: return no data back to host */ // and ignores the input
  }

  // This function changes our feature report, and is called from V-USB library after we return USB_NO_MSG from the function above.
  uchar usbFunctionWrite(uchar *data, uchar len)
  {

    if (len > bytesToRead)
      len = bytesToRead;
    memcpy(writeBuf, data, len);

    writeBuf += len;
    bytesToRead-= len;
    bool isEnd = bytesToRead == 0;

    if (isEnd) [[likely]]
    {
      switch (receivedData.ReportId)
      {
      case LAMP_ATTRIBUTES_REQUEST_REPORT_ID:
      {
        UpdateRequestLampFromLampAttributesRequestReport(receivedData.attributesRequest);
        break;
      }
      case LAMP_MULTI_UPDATE_REPORT_ID:
      {
        UpdateLampStateCacheFromMultiUpdateReport(receivedData.multiUpdate);
        break;
      }
      case LAMP_RANGE_UPDATE_REPORT_ID:
      {
        UpdateLampStateCacheFromRangeUpdateReport(receivedData.rangeUpdate);
        break;
      }
      case LAMP_ARRAY_CONTROL_REPORT_ID:
      {
        ProcessControlReport(receivedData.control);
        break;
      }
      }
    }

    return (uchar)isEnd; /* return 1 if this was the last chunk */
  }
#ifdef __cplusplus
} // extern "C"
#endif
