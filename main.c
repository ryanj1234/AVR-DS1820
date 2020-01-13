// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* \li File:               main.c
* \li Compiler:           IAR EWAAVR 3.20a
* \li Support mail:       avr@atmel.com
*
* \li Supported devices:  All AVRs.
*
* \li Application Note:   AVR318 - Dallas 1-Wire(R) master.
*                         
*
* \li Description:        Example on how to use the 1-Wire(R) interface
*                         master.
*
*                         $Revision: 1.7 $
*                         $Date: Thursday, August 19, 2004 14:27:18 UTC $
****************************************************************************/

#include <avr/io.h>
#include <util/delay.h>
// #include <inavr.h>

#include "OWIPolled.h"
#include "OWIHighLevelFunctions.h"
#include "OWIBitFunctions.h"
#include "OWIcrc.h"

#include <string.h> // Used for memcpy.
#include <stdio.h>

#include "../uart/uart.h"


// Defines used only in code example.
#define DS1820_FAMILY_ID                0x28 
#define DS1820_START_CONVERSION         0x44
#define DS1820_READ_SCRATCHPAD          0xbe
#define DS1820_ERROR                    -1000   // Return code. Outside temperature range.

#define DS2890_FAMILY_ID                0x2c
#define DS2890_WRITE_CONTROL_REGISTER   0X55
#define DS2890_RELEASE_CODE             0x96
#define DS2890_WRITE_POSITION           0x0f

#define SEARCH_SUCCESSFUL               0x00
#define SEARCH_CRC_ERROR                0x01

#define FALSE       0
#define TRUE        1

#define MAX_DEVICES 8       //!< Max number of devices to search for.

#define BUSES   (OWI_PIN_4 | OWI_PIN_5) //!< Buses to search.


/*! \brief  Data type used to hold information about slave devices.
 *  
 *  The OWI_device data type holds information about what bus each device
 *  is connected to, and its 64 bit identifier.
 */
typedef struct
{
    unsigned char bus;      //!< A bitmask of the bus the device is connected to.
    unsigned char id[8];    //!< The 64 bit identifier.
} OWI_device;


// Prototypes of functions used in exemples.
unsigned char SearchBuses(OWI_device * devices, unsigned char len, unsigned char buses);
OWI_device * FindFamily(unsigned char familyID, OWI_device * devices, unsigned char size);
signed int DS1820_ReadTemperature(unsigned char bus, unsigned char * id);


/*! \brief  Example application for the polled drivers.
 *
 *  Example application for the software only and polled UART driver.
 *  This example application will find all devices (upper bounded by MAX_DEVICES) 
 *  on the buses defined by BUSES. It then tries to find either a DS1820 or DS2890 
 *  device on a bus, and communicate with them to read temperature (DS1820) or 
 *  set wiper position (DS2890).
 *  This example is not written in a very optimal way. It is merely intended to show
 *  how the polled 1-Wire(R) driver can be used.
 */
int main(void)
{
    OWI_device ds1820 = {
        OWI_PIN_4,
        {0x28, 0xAA, 0xC1, 0xF9, 0x54, 0x14, 0x01, 0xEC}
    };
    OWI_device* pds1820 = &ds1820;
    signed int temperature = 0;

    uart_init(9600);

    OWI_Init(BUSES);
    
    // Do something useful with the slave devices in an eternal loop.
    for (;;)
    {
        // If there is a DS1820 temperature sensor on a bus, read the
        // temperature.
        // The DS1820 must have Vdd pin connected for this code to work.
        if (pds1820 != NULL)
        {
            temperature = DS1820_ReadTemperature(ds1820.bus, ds1820.id);
            signed int tmp = ((temperature & 0xfff0) << 3) - 32 + (((0x10 - 0x1F) << 7) / 0x10);
            float t = (float)tmp * 0.0140625 + 32;
            printf("Temperature read: %.2f (deg F)\n", t);
        }
        else
        {
            printf("No device found!\n");
        }
        
    }
}


/*! \brief  Perform a 1-Wire search
 *
 *  This function shows how the OWI_SearchRom function can be used to 
 *  discover all slaves on the bus. It will also CRC check the 64 bit
 *  identifiers.
 *
 *  \param  devices Pointer to an array of type OWI_device. The discovered 
 *                  devices will be placed from the beginning of this array.
 *
 *  \param  len     The length of the device array. (Max. number of elements).
 *
 *  \param  buses   Bitmask of the buses to perform search on.
 *
 *  \retval SEARCH_SUCCESSFUL   Search completed successfully.
 *  \retval SEARCH_CRC_ERROR    A CRC error occured. Probably because of noise
 *                              during transmission.
 */
unsigned char SearchBuses(OWI_device * devices, unsigned char len, unsigned char buses)
{
    unsigned char i, j;
    unsigned char presence;
    unsigned char * newID;
    unsigned char * currentID;
    unsigned char currentBus;
    unsigned char lastDeviation;
    unsigned char numDevices;
    
    // Initialize all addresses as zero, on bus 0 (does not exist).
    // Do a search on the bus to discover all addresses.    
    for (i = 0; i < len; i++)
    {
        devices[i].bus = 0x00;
        for (j = 0; j < 8; j++)
        {
            devices[i].id[j] = 0x00;
        }
    }
    
    // Find the buses with slave devices.
    presence = OWI_DetectPresence(BUSES);
    
    numDevices = 0;
    newID = devices[0].id;
    
    // Go through all buses with slave devices.
    for (currentBus = 0x01; currentBus; currentBus <<= 1)
    {
        lastDeviation = 0;
        currentID = newID;
        if (currentBus & presence) // Devices available on this bus.
        {
            // Do slave search on each bus, and place identifiers and corresponding
            // bus "addresses" in the array.
            do  
            {
                memcpy(newID, currentID, 8);
                OWI_DetectPresence(currentBus);
                lastDeviation = OWI_SearchRom(newID, lastDeviation, currentBus);
                currentID = newID;
                devices[numDevices].bus = currentBus;
                numDevices++;
                newID=devices[numDevices].id;                
            }  while(lastDeviation != OWI_ROM_SEARCH_FINISHED);            
        }
    }

    // Go through all the devices and do CRC check.
    for (i = 0; i < numDevices; i++)
    {
        // If any id has a crc error, return error.
        if(OWI_CheckRomCRC(devices[i].id) != OWI_CRC_OK)
        {
            return SEARCH_CRC_ERROR;
        }
    }
    // Else, return Successful.
    return SEARCH_SUCCESSFUL;
}

/*! \brief  Find the first device of a family based on the family id
 *
 *  This function returns a pointer to a device in the device array
 *  that matches the specified family.
 *
 *  \param  familyID    The 8 bit family ID to search for.
 *
 *  \param  devices     An array of devices to search through.
 *
 *  \param  size        The size of the array 'devices'
 *
 *  \return A pointer to a device of the family.
 *  \retval NULL    if no device of the family was found.
 */
OWI_device * FindFamily(unsigned char familyID, OWI_device * devices, unsigned char size)
{
    unsigned char i = 0;
    
    // Search through the array.
    while (i < size)
    {
        // Return the pointer if there is a family id match.
        if ((*devices).id[0] == familyID)
        {
            return devices;
        }
        devices++;
        i++;
    }
    // Else, return NULL.
    return NULL;
}


/*! \brief  Read the temperature from a DS1820 temperature sensor.
 *
 *  This function will start a conversion and read back the temperature
 *  from a DS1820 temperature sensor.
 *
 *  \param  bus A bitmask of the bus where the DS1820 is located.
 *  
 *  \param  id  The 64 bit identifier of the DS1820.
 *
 *  \return The 16 bit signed temperature read from the DS1820.
 */
signed int DS1820_ReadTemperature(unsigned char bus, unsigned char * id)
{
    signed int temperature;
    
    // Reset, presence.
    if (!OWI_DetectPresence(bus))
    {
        return DS1820_ERROR; // Error
    }
    // Match the id found earlier.
    OWI_MatchRom(id, bus);
    // Send start conversion command.
    OWI_SendByte(DS1820_START_CONVERSION, bus);
    // Wait until conversion is finished.
    // Bus line is held low until conversion is finished.
    while (!OWI_ReadBit(bus))
    {
    
    }
    // Reset, presence.
    if(!OWI_DetectPresence(bus))
    {
        return -1000; // Error
    }
    // Match id again.
    OWI_MatchRom(id, bus);
    // Send READ SCRATCHPAD command.
    OWI_SendByte(DS1820_READ_SCRATCHPAD, bus);
    // Read only two first bytes (temperature low, temperature high)
    // and place them in the 16 bit temperature variable.
    temperature = OWI_ReceiveByte(bus);
    temperature |= (OWI_ReceiveByte(bus) << 8);
    
    return temperature;
}
