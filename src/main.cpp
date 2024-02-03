/*
 * Copyright (c) 2024 Matej Kocourek
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#define DISABLEMILLIS
#include <Arduino.h>
#include <usbdrv.h>
#include <avr/power.h>
#include <util/delay.h>
#include <SK6812_io.cpp>
#include "lampArrayHid.h"



/********EASY SETUP*********/
//Set the PIN the strip is on. Not tested anything besides 8.
constexpr uint8_t pinStrip = 8u;
//How many LEDs there is per meter. This is usually the parameter of the product (30, 60, ...).
constexpr uint16_t lampsPerMeter = 60;
//How many LEDs is connected to this one strip.
constexpr uint16_t m_lampCount = 37u;

///This section is used to perform colorspace conversion. We assume (standard is vague) that the PC sends the values in 6500K (usual for LED monitors).
//Relative to 6500K, what is the most bright color the LED strip can produce by combining RGB diodes.
//(If we also assume that the RGB LEDs produce 6500K white, all values are 255.)
constexpr uint8_t white_rgb_r = 255;
constexpr uint8_t white_rgb_g = 255;
constexpr uint8_t white_rgb_b = 255;
//Relative to 6500K, what is the most bright color the LED strip can produce only by its white LED.
//You can use this page to get the value for the LED strip you bought: https://academo.org/demos/colour-temperature-relationship/
//The values here correspond to 4000K (natural white).
constexpr uint8_t white_white_r = 255;
constexpr uint8_t white_white_g = 206;
constexpr uint8_t white_white_b = 176;
/***************************/

/********SETUP*********/
//If you used easy setup above, no need to change these values. They will be calculated in compile time.

///The LampArray standard expects to know where each LED is located. We need to provide bounding box.
constexpr float m_lampSpacing = 1000.0/lampsPerMeter * 1000ul;
//The project assumes one LED strip placed horizontally (with the first LED being on the left).
constexpr uint32_t m_boundingBoxWidthInMicrometers = m_lampCount * m_lampSpacing;
constexpr uint32_t m_boundingBoxHeightInMicrometers = 0ul;
constexpr uint32_t m_boundingBoxDepthInMicrometers = 0ul;
///The minimal time it takes before all LEDs change color (their resposivness + communication overhead).
//It might be possible to get smoother effects by decreasing this value, but it comes with the cost of USB stability.
constexpr uint32_t m_minUpdateInternalInMicroseconds = 33333ul;

//Information about what type of device this is. Use chassis for things inside the PC case, furniture for desk lighting, peripheral for keyboard/monitor, etc.
constexpr LampArrayKind m_kind = LampArrayKind::LampArrayKindPeripheral;

PROGMEM const LampArrayAttributesReport lampArrayAttributesReport{
  .ReportId = LAMP_ARRAY_ATTRIBUTES_REPORT_ID,
  .LampCount = m_lampCount,
  .BoundingBoxWidthInMillimeters = m_boundingBoxWidthInMicrometers,
  .BoundingBoxHeightInMillimeters = m_boundingBoxHeightInMicrometers,
  .BoundingBoxDepthInMillimeters = m_boundingBoxDepthInMicrometers,
  .LampArrayKind = m_kind,
  .MinUpdateIntervalInMicroseconds = m_minUpdateInternalInMicroseconds
};
/********************/

//This is the format the LED strip expects the data to arrive. Do not change structure.
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

/// @brief Used to perform gamma correction over LED brightness, as they respon non-linearly
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

/// @brief Structure for operations over individual LED, and remembering the last state. Can be inherited or changed.
struct MyLed
{
  /// @brief Sets attributes that are the same for all LEDs
  /// @param attributes The attributes are set here
  static constexpr void exportGeneralAttributes(LampAttributes &attributes)
  {
    attributes.UpdateLatencyInMicroseconds = m_minUpdateInternalInMicroseconds; // 2500;//400Hz
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
  /// @brief Sets attributes for individual LED based on its index
  /// @param lampId The index of the LED
  /// @param attributes The attributes are set here
  static constexpr void exportIndividualAttributes(uint16_t lampId, LampAttributes &attributes)
  {
    attributes.LampId = lampId;
    attributes.PositionXInMillimeters = lampId * (m_boundingBoxWidthInMicrometers / m_lampCount);
  }
  /// @brief Set the color of this LED
  /// @param lampArrayColor The color to set it to
  /// @return If there was a change performing this operation (false if setting to the same color as was before).
  bool set(const LampArrayColor &lampArrayColor)
  {
    return set(lampArrayColor.RedChannel, lampArrayColor.GreenChannel, lampArrayColor.BlueChannel);
  }

private:
  /// @brief Performs gamma correction over one channel
  /// @param color The color to correct
  /// @return Gamma-corrected value, better representing human perceived brightness
  static uint8_t gammaCorrect(uint8_t color)
  {
    return pgm_read_byte(&gamma8[color]);
  }
  /// @brief Converts RGB colorspace to RGBW.
  /// @param r Red channel of the source colorspace
  /// @param g Green channel of the source colorspace
  /// @param b Blue channel of the source colorspace
  /// @return RGBW colorspace.
  static constexpr GRBW RGBtoRGBW(uint8_t r, uint8_t g, uint8_t b)
  {
    uint8_t w_out = smallest(r, g, b);

    constexpr float r_a = 1.0 * white_white_r / white_rgb_r;
    constexpr float g_a = 1.0 * white_white_g / white_rgb_g;
    constexpr float b_a = 1.0 * white_white_b / white_rgb_b;

    uint8_t r_out = r - (uint8_t)(w_out * r_a);
    uint8_t g_out = g - (uint8_t)(w_out * g_a);
    uint8_t b_out = b - (uint8_t)(w_out * b_a);

    return GRBW{
        .G = g_out,
        .R = r_out,
        .B = b_out,
        .W = w_out};
  }

  /// @brief Performs gamma correction over all channels
  /// @param in The color to correct
  /// @return Gamma-corrected value, better representing human perceived brightness
  static GRBW gammaCorrect(GRBW in)
  {
    return GRBW{
        .G = gammaCorrect(in.G),
        .R = gammaCorrect(in.R),
        .B = gammaCorrect(in.B),
        .W = gammaCorrect(in.W)};
  }
  /// @brief Sets the color of this LED directly knowing all channels.
  /// @param R R
  /// @param G G
  /// @param B B
  /// @param W W
  /// @return If there was a change performing this operation (false if setting to the same color as was before).
  bool set(uint8_t R, uint8_t G, uint8_t B, uint8_t W)
  {
    bool changed = color.R != R || color.G != G || color.B != B || color.W != W;
    color.R = R;
    color.G = G;
    color.B = B;
    color.W = W;

    return changed;
  }
  /// @brief  Sets the color of this LED directly knowing RGB.
  /// @param r R
  /// @param g G
  /// @param b B
  /// @return If there was a change performing this operation (false if setting to the same color as was before).
  bool set(uint8_t r, uint8_t g, uint8_t b)
  {
    auto newColor = gammaCorrect(RGBtoRGBW(r, g, b));
    bool res = color != newColor;
    color = newColor;
    return res;
  }

  GRBW color;
};

//If having multiple LED classes, it can be specified which is gonna be used.
typedef MyLed LED;

/// @brief Class to operate over one connected LED strip.
struct LEDStrip
{
  LEDStrip()
  {
    _pin_mask = digitalPinToBitMask(pinStrip);
    _port = portOutputRegister(digitalPinToPort(pinStrip));
    _port_reg = portModeRegister(digitalPinToPort(pinStrip));

    memset(leds,0,sizeof(leds));
  }

  //There is no need to update the whole strip if only one LED changed. Keep the index of the furthest LED that needs to be changed.
  int16_t updateLedsToIndex = -1;
  LED leds[m_lampCount];

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

    StripSend sendingProcess(_pin_mask, (uint8_t *)_port, (uint8_t *)_port_reg);
    size_t size = sizeof(*leds) * (updateLedsToIndex + 1);
    for (size_t i = 0; i < size; i++)
    {
      sendingProcess.sendByte(((uint8_t *)leds)[i]);
    }

    updateLedsToIndex = -1;
  }

} ledStrip;

//Needed for holding state of HID responses.
uint16_t m_lastLampIdRequested = 0;
bool m_isAutonomousMode = true;//Does nothing. If set to true, the PC relinquishes the control of the strip.
LampAttributesResponseReport lampAttributeReport;
bool updateLeds = false;

void setup()
{
  //Turn of useless components.
  ADCSRA &= ~(1 << ADEN);
  power_adc_disable();
  power_spi_disable();
  power_twi_disable();

  //Set attributes that are the same only here, once.
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

/// @brief Marks the report as ready for sending.
/// @tparam T The type of the report. Is implicitly guessed.
/// @param data The report structure (in RAM). The lifetime of this must be at least until the USB sends the data.
/// @return The length of the message. Pass this on to the USB handler.
template <typename T>
usbMsgLen_t SendFeatureReport(const T &data) noexcept
{
  usbMsgPtr = (uchar *)&data;
  usbMsgFlags &= ~USB_FLG_MSGPTR_IS_ROM;//Is in RAM, not in ROM
  return sizeof(data);
}


usbMsgLen_t SendLampArrayAttributesReport() noexcept
{
  usbMsgPtr = (uchar *)&lampArrayAttributesReport;
  usbMsgFlags |= USB_FLG_MSGPTR_IS_ROM;//Is in ROM, to save on memory. Must be specified explicitly.
  return sizeof(lampArrayAttributesReport);
}

usbMsgLen_t SendLampAttributesReport() noexcept
{
  LED::exportIndividualAttributes(m_lastLampIdRequested, lampAttributeReport.Attributes);

  m_lastLampIdRequested = (m_lastLampIdRequested + 1) % m_lampCount;

  return SendFeatureReport(lampAttributeReport);
}


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

void ProcessControlReport(const LampArrayControlReport &report) noexcept
{
  m_isAutonomousMode = !!report.AutonomousMode;
}

void loop()
{
  usbPoll();//Always poll USB as often as possible

  if (updateLeds) //If the poll resulted in a request for update
  {
    updateLeds = false;

    //Very important when using V-USB! During the time that the LED strip will be updated,
    //all interrupts need to be disabled. Because of that, we have to be sure no USB communication
    //is taking place during this time, or the device permanently disconnects/stops reponding (no LampArray device available).

    //This is a little bit of a hack, because the PC has to wait the minimum specified time before sending another
    //update to the LampArray. During this time, the lanes should be quiet. We perform the risky update here.
    //Wait too little, and the device is still sending the data back. Wait too much, and the next poll is taking place.
    //If having problems with disconnecting, try tuning the delay here (before calling the update strip).
    uint8_t timeoutCounter = 0;
    do
    {
      usbPoll();
    } while (--timeoutCounter);
    //_delay_ms(1);
    ledStrip.updateStrip();
  }
}

/// @brief Contains every type that we receive, to save space
union ReceivedData
{
  uint8_t ReportId;//When data is received, one can tell the type by checking this id
  LampAttributesRequestReport attributesRequest;
  LampMultiUpdateReport multiUpdate;
  LampRangeUpdateReport rangeUpdate;
  LampArrayControlReport control;
} receivedData;

/// @brief This is how HID request looks like.
struct hidRequest
{
  uint8_t bmRequestType;
  uint8_t bRequest;
  uint8_t ReportID;
  uint8_t ReportType;
  usbWord_t wIndex;
  usbWord_t wLength;
};

uint16_t bytesToRead;
uchar *writeBuf;

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
          bytesToRead = rq->wLength.word;
          writeBuf = (unsigned char *)&receivedData;
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
    bytesToRead -= len;
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
