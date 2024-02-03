#pragma once
#include <Arduino.h>


// HID Report Descriptor
PROGMEM const uint8_t usbHidReportDescriptor[] = {
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
    uint16_t LampId;//The ID of the lamp for which these attributes apply
    uint32_t PositionXInMillimeters;//Positions in micrometers, must be in the bounding box
    uint32_t PositionYInMillimeters;
    uint32_t PositionZInMillimeters;
    uint32_t UpdateLatencyInMicroseconds;//Each LED can have different update latency.
    uint32_t LampPurposes;//The purpose of this LED, as per enum LampPurposeKind
    uint8_t RedLevelCount;//The amount of unique red levels this LED can reproduce.
    uint8_t GreenLevelCount;//The amount of unique green levels this LED can reproduce.
    uint8_t BlueLevelCount;//The amount of unique blue levels this LED can reproduce.
    uint8_t IntensityLevelCount;//The amount of unique brightness levels this LED supports, to each RGB color from above. (Is 1 for RGB diodes)
    uint8_t IsProgrammable;//Can change color
    uint8_t LampKey;//Keybord key that is mapped to this LED. If not a keyboard, must be 0.
};

#define LAMP_ARRAY_ATTRIBUTES_REPORT_ID 1
struct __attribute__ ((__packed__)) LampArrayAttributesReport//From device to PC (when requested). Descibes the configuration of the LED strip.
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
struct __attribute__ ((__packed__)) LampAttributesRequestReport//From PC to device. The PC sets the iterator to this ID. (Is usually followed by a request for the selected LED).
{
    uint8_t ReportId;
    uint16_t LampId;
};

#define LAMP_ATTRIBUTES_RESPONSE_REPORT_ID 3
struct __attribute__ ((__packed__)) LampAttributesResponseReport//From device to PC (when requested). Describes the configuration of the selected LED.
{
    uint8_t ReportId;
    LampAttributes Attributes;
};

#define LAMP_MULTI_UPDATE_REPORT_ID 4
#define LAMP_MULTI_UPDATE_LAMP_COUNT 8
struct __attribute__ ((__packed__)) LampMultiUpdateReport//From PC to device. Sets arbitrary LEDs to some colors.
{
    uint8_t ReportId;
    uint8_t LampCount;
    uint8_t LampUpdateFlags;//If the strip should be updated now, or wait for the rest.
    uint16_t LampIds[LAMP_MULTI_UPDATE_LAMP_COUNT];
    LampArrayColor UpdateColors[LAMP_MULTI_UPDATE_LAMP_COUNT];
};


#define LAMP_RANGE_UPDATE_REPORT_ID 5
struct __attribute__ ((__packed__)) LampRangeUpdateReport//From PC to device - sets fixed color to a consecutive range of LEDs.
{
    uint8_t ReportId;
    uint8_t LampUpdateFlags;//If the strip should be updated now, or wait for the rest.
    uint16_t LampIdStart;
    uint16_t LampIdEnd;
    LampArrayColor UpdateColor;
};

#define LAMP_ARRAY_CONTROL_REPORT_ID 6
struct __attribute__ ((__packed__)) LampArrayControlReport//From PC to device - specifies if the PC is in charge of the strip, or the device can do whatever it wants.
{
    uint8_t ReportId;
    uint8_t AutonomousMode;
};