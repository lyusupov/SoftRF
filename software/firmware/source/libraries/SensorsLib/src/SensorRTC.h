/**
 *
 * @license MIT License
 *
 * Copyright (c) 2023 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      SensorRTC.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-07
 *
 */
#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stdint.h>
#endif /*defined(ARDUINO)*/

#include <sys/time.h>
#include "SensorPlatform.hpp"

typedef enum {
    DT_FMT_HM,          // Format Style : Hour:Minute
    DT_FMT_HMS,         // Format Style : Hour:Minute:Second
    DT_FMT_YMD,         // Format Style : Year-Month-Day
    DT_FMT_MDY,         // Format Style : Month-Day-Year
    DT_FMT_DMY,         // Format Style : Day-Month-Year
    DT_FMT_YMD_HMS,     // Format Style : Year-Month-Day/Hour:Minute:Second
    DT_FMT_YMD_HMS_WEEK // Format Style : Year-Month-Day/Hour:Minute:Second - Weekday
} DateTimeFormat;

class RTC_DateTime
{
    enum Month {
        JANUARY = 1,
        FEBRUARY,
        MARCH,
        APRIL,
        MAY,
        JUNE,
        JULY,
        AUGUST,
        SEPTEMBER,
        OCTOBER,
        NOVEMBER,
        DECEMBER
    };

public:
    RTC_DateTime() : year(0), month(0), day(0), hour(0), minute(0), second(0), week(0) {}

    RTC_DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint8_t week = 0) :
        year(year), month(month), day(day), hour(hour), minute(minute), second(second), week(week) {}

    RTC_DateTime(struct tm info) : year(info.tm_year + 1900), month(info.tm_mon + 1), day(info.tm_mday),
        hour(info.tm_hour), minute(info.tm_min), second(info.tm_sec), week(info.tm_wday) {}

    RTC_DateTime(const char *date, const char *time)
    {
        if (date == nullptr || time == nullptr) {
            year = 0;
            month = 0;
            day = 0;
            hour = 0;
            minute = 0;
            second = 0;
            week = 0;
            return;
        }
        // sample input: date = "Dec 26 2009", time = "12:34:56"
        year = 2000 + parseStringToUint8(date + 9);
        switch (date[0]) {
        case 'J':
            if ( date[1] == 'a' )
                month = JANUARY;
            else if ( date[2] == 'n' )
                month = JUNE;
            else
                month = JULY;
            break;
        case 'F':
            month = FEBRUARY;
            break;
        case 'A':
            month = date[1] == 'p' ? APRIL : AUGUST;
            break;
        case 'M':
            month = date[2] == 'r' ? MARCH : MAY;
            break;
        case 'S':
            month = SEPTEMBER;
            break;
        case 'O':
            month = OCTOBER;
            break;
        case 'N':
            month = NOVEMBER;
            break;
        case 'D':
            month = DECEMBER;
            break;
        }
        day = parseStringToUint8(date + 4);
        hour = parseStringToUint8(time);
        minute = parseStringToUint8(time + 3);
        second = parseStringToUint8(time + 6);
    }

    bool operator==(RTC_DateTime d)
    {
        return ((d.year == year) && (d.month == month) && (d.day == day)
                && (d.hour == hour) && (d.minute == minute));
    }

    struct tm toUnixTime()
    {
        struct tm t_tm;
        t_tm.tm_hour = hour;
        t_tm.tm_min = minute;
        t_tm.tm_sec = second;
        t_tm.tm_year = year - 1900;    //Year, whose value starts from 1900
        t_tm.tm_mon = month - 1;       //Month (starting from January, 0 for January) - Value range is [0,11]
        t_tm.tm_mday = day;
        t_tm.tm_wday = week;
        return t_tm;
    }

    // *INDENT-OFF*
    uint16_t getYear() const { return year; }
    uint8_t  getMonth() const { return month; }
    uint8_t  getDay() const { return day; }
    uint8_t  getHour() const { return hour; }
    uint8_t  getMinute() const { return minute; }
    uint8_t  getSecond() const { return second; }
    uint8_t  getWeek() const { return week; }
    // char getAMPM() const { return AMPM; }
    // bool isAvailable() const { return available; }
    // *INDENT-ON*

private:
    // *INDENT-OFF*
    // void setAMPM(char ampm) { AMPM = ampm; }
    // void setAvailable(bool avail) { available = avail; }
    // *INDENT-ON*

    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t week;
    // char AMPM;
    // bool available;

    uint8_t parseStringToUint8(const char *pString)
    {
        uint8_t value = 0;

        // skip leading 0 and spaces
        while ('0' == *pString || *pString == ' ') {
            pString++;
        }

        // calculate number until we hit non-numeral char
        while ('0' <= *pString && *pString <= '9') {
            value *= 10;
            value += *pString - '0';
            pString++;
        }
        return value;
    }
};

class RTC_Alarm
{
public:
    RTC_Alarm(void): second(0), minute(0), hour(0), day(0), week(0) {}
    RTC_Alarm(uint8_t hour, uint8_t minute, uint8_t second, uint8_t day, uint8_t week ) :
        second(second), minute(minute), hour(hour), day(day), week(week) {}

    // *INDENT-OFF*
    uint8_t  getDay() const { return day; }
    uint8_t  getHour() const { return hour; }
    uint8_t  getMinute() const { return minute; }
    uint8_t  getSecond() const { return second; }
    uint8_t  getWeek() const { return week; }
    // *INDENT-ON*

private:
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t week;
};

class SensorRTC
{
public:

#if defined(ARDUINO)
    /**
     * @brief Initialize the RTC device on the Arduino platform using I2C.
     *
     * This function is used to initialize the RTC device when the code is running on an Arduino platform.
     * It takes an I2C bus object (`TwoWire`), and the pin numbers for the SDA (Serial Data Line) and SCL (Serial Clock Line)
     * as parameters. The function attempts to establish communication with the RTC device over the specified I2C bus.
     *
     * @param wire A reference to the `TwoWire` object representing the I2C bus.
     * @param sda The GPIO pin number used for the SDA line.
     * @param scl The GPIO pin number used for the SCL line.
     * @return `true` if the initialization is successful, `false` otherwise.
     */
    virtual bool begin(TwoWire &wire, int sda, int scl) = 0;

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)

    /**
     * @brief Initialize the RTC device on the ESP platform using legacy I2C.
     *
     * This function is used to initialize the RTC device when the code is running on an ESP platform
     * and the legacy I2C interface is being used. It takes the I2C port number, and the pin numbers for
     * the SDA and SCL lines as parameters. The function tries to set up communication with the RTC device
     * using the specified I2C port and pins.
     *
     * @param port_num The I2C port number to use for communication.
     * @param sda The GPIO pin number used for the SDA line.
     * @param scl The GPIO pin number used for the SCL line.
     * @return `true` if the initialization is successful, `false` otherwise.
     */
    virtual bool begin(i2c_port_t port_num, int sda, int scl) = 0;
#else

    /**
     * @brief Initialize the RTC device on the ESP platform using an I2C master bus handle.
     *
     * This function is used to initialize the RTC device when the code is running on an ESP platform
     * and an I2C master bus handle is provided. It takes the I2C master bus handle as a parameter
     * and attempts to establish communication with the RTC device using this handle.
     *
     * @param handle The I2C master bus handle to use for communication.
     * @return `true` if the initialization is successful, `false` otherwise.
     */
    virtual bool begin(i2c_master_bus_handle_t handle) = 0;
#endif
#endif // defined(ARDUINO)

    /**
     * @brief Initialize the RTC device using a custom callback.
     *
     * This function allows for a custom way of initializing the RTC device. It takes a custom callback function
     * as a parameter. The callback can be used to perform custom communication or configuration operations
     * during the initialization process.
     *
     * @param callback A pointer to the custom callback function. The callback should follow the signature
     *                 defined in `SensorCommCustom::CustomCallback`.
     * @return `true` if the initialization is successful, `false` otherwise.
     */
    virtual bool begin(SensorCommCustom::CustomCallback callback) = 0;

    /**
    * @brief Set the date and time of the RTC using an RTC_DateTime object.
    *
    * This is a pure virtual function that derived classes must implement.
    * It allows setting the RTC's date and time using a single RTC_DateTime object.
    *
    * @param datetime An RTC_DateTime object containing the date and time information to set on the RTC.
    */
    virtual void setDateTime(RTC_DateTime datetime) = 0;

    /**
    * @brief Get the current date and time from the RTC.
    *
    * This is a pure virtual function that derived classes must implement.
    * It retrieves the current date and time from the RTC and returns them as an RTC_DateTime object.
    *
    * @return An RTC_DateTime object representing the current date and time of the RTC.
    */
    virtual RTC_DateTime getDateTime() = 0;

    /**
    * @brief Get the name of the RTC chip.
    *
    * This is a pure virtual function that derived classes must implement.
    * It returns a pointer to a null - terminated string representing the name of the RTC chip.
    *
    * @return A pointer to a constant character array containing the name of the RTC chip.
    */
    virtual const char *getChipName() = 0;


    /**
    * @brief Retrieve the current date and time from the RTC (Real-Time Clock) and populate a struct tm object.
    *
    * This function fetches the current date and time information from the RTC device.
    * It then fills the provided `struct tm` object with the corresponding values.
    * The `struct tm` object is a standard C structure used to represent date and time components,
    * such as year, month, day, hour, minute, and second.
    *
    * @param info A pointer to a `struct tm` object where the retrieved date and time information will be stored.
    *             This pointer must not be a null pointer
    *
    * @note The year value in the `struct tm` object is the number of years since 1900.
    *       The month value ranges from 0 (January) to 11 (December).
    *       The day value represents the day of the month, starting from 1.
    *       The hour value ranges from 0 to 23, the minute and second values range from 0 to 59.
    */
    void getDateTime(struct tm*info)
    {
        if (!info)return;
        RTC_DateTime datetime = getDateTime();
        *info = datetime.toUnixTime();
    }

    /**
    * @brief Set the date and time of the RTC using individual values.
    * @param year The year to set on the RTC. It's usually a 16 - bit unsigned integer.
    * @param month The month to set on the RTC, ranging from 1 (January) to 12 (December).
    * @param day The day of the month to set on the RTC, ranging from 1 to 31, depending on the month and leap year.
    * @param hour The hour to set on the RTC, ranging from 0 to 23.
    * @param minute The minute to set on the RTC, ranging from 0 to 59.
    * @param second The second to set on the RTC, ranging from 0 to 59.
    */
    void setDateTime(uint16_t year, uint8_t month, uint8_t day,
                     uint8_t hour, uint8_t minute, uint8_t second)
    {
        setDateTime(RTC_DateTime(year, month, day, hour, minute, second));
    }

    /**
     * @brief Format the current date and time according to the specified style.
     *
     * This function retrieves the current date and time from the RTC (Real-Time Clock)
     * using the `getDateTime()` function. Then, it formats the date and time
     * into a string based on the provided style. The result is stored in a static buffer
     * and a pointer to this buffer is returned.
     *
     * @param style The format style for the date and time string. It should be one of the
     *              values defined in the `DateTimeFormat` enumeration. The default value
     *              is `DT_FMT_YMD_HMS_WEEK`, which represents the format "Year-Month-Day/Hour:Minute:Second - Weekday".
     * @return A pointer to a static character array containing the formatted date and time string.
     *         Note that the buffer is static, so subsequent calls to this function will overwrite
     *         the previous result.
     */
    const char *strftime(DateTimeFormat style = DT_FMT_YMD_HMS_WEEK)
    {
        const char *formatStr = NULL;
        static char format[FORMAT_BUFFER_SIZE];

        RTC_DateTime t = getDateTime();

        switch (style) {
        case DT_FMT_HM:
            formatStr = "%02d:%02d";
            break;
        case DT_FMT_HMS:
            formatStr = "%02d:%02d:%02d";
            break;
        case DT_FMT_YMD:
            formatStr = "%d-%02d-%02d";
            break;
        case DT_FMT_MDY:
            formatStr = "%02d-%02d-%d";
            break;
        case DT_FMT_DMY:
            formatStr = "%02d-%02d-%d";
            break;
        case DT_FMT_YMD_HMS:
            formatStr = "%d-%02d-%02d/%02d:%02d:%02d";
            break;
        case DT_FMT_YMD_HMS_WEEK:
            formatStr = "%d-%02d-%02d/%02d:%02d:%02d - %s";
            break;
        default:
            formatStr = "%02d:%02d";
            break;
        }
        int result = formatDateTime(formatStr, format, FORMAT_BUFFER_SIZE, &t);
        if (result < 0 || (size_t)result >= FORMAT_BUFFER_SIZE) {
            format[0] = '\0';
        }
        return format;
    }

    /**
    * @brief Read the time from the RTC (Real-Time Clock) and write it to the system clock.
    *
    * This function retrieves the current date and time from the RTC chip,
    * converts it to a Unix timestamp, and then sets this timestamp as the system time
    * on systems that support it (e.g., BSD systems).
    *
    * @param tz A pointer to a timezone structure specifying timezone information.
    *           In modern systems, this parameter is usually ignored and can be set to NULL.
    *           The default value of this parameter is NULL.
    * @return On success, returns the Unix timestamp (in seconds) read from the RTC and converted;
    *         if the mktime call fails during the conversion process, returns -1;
    *         if the settimeofday call fails on a supported system, an error log is recorded,
    *         but the timestamp is still returned.
    */
    time_t hwClockRead(const timezone *tz = NULL)
    {
        struct timeval val;
        // Retrieve the date and time from the RTC chip and convert it to a struct tm structure
        struct tm t_tm = getDateTime().toUnixTime();
        // Convert the struct tm structure to a Unix timestamp
        time_t timestamp = mktime(&t_tm);
        // Check if the mktime conversion was successful
        if (timestamp == -1) {
            log_e("mktime failed");
            return -1;
        }
        val.tv_sec = timestamp;
        val.tv_usec = 0;
#if __BSD_VISIBLE
        // On supported BSD systems, attempt to set the new time to the system clock
        if (settimeofday(&val, tz) == -1) {
            log_e("settimeofday failed");
        }
#endif /*__BSD_VISIBLE*/
        // Return the converted Unix timestamp
        return val.tv_sec;
    }

    /**
    * @brief Write the current system time to the RTC clock chip.
    * @return If the system time is not obtained (time call) or the time is not converted (localtime_r call), it returns -1;
    * If the time is successfully obtained, converted and set to the RTC chip, it returns 0
    */
    int hwClockWrite()
    {
        time_t now;
        struct tm  info;
        // Get the current system time as a Unix timestamp
        now = time(NULL);
        if (now == -1) {
            log_e("time failed");
            return -1;
        }
        if (localtime_r(&now, &info) == NULL) {
            log_e("localtime_r failed");
            return -1;
        }
        setDateTime(info);
        return 0;
    }

    /**
     * Calculate the day of the week for a given date using Zeller's congruence.
     *
     * @param day The day of the month (1 - 31).
     * @param month The month of the year (1 - 12).
     * @param year The year.
     * @return The day of the week, where 0 represents Sunday, 1 represents Monday, ..., 6 represents Saturday.
     *         Returns 0xFF if the input date is invalid.
     */
    uint32_t getDayOfWeek(uint32_t day, uint32_t month, uint32_t year)
    {
        if (day < 1 || day > 31 || month < 1 || month > 12) {
            return 0xFF;
        }
        if (month < 3) {
            month += 12;
            year--;
        }
        // Evaluate the parts of Zeller's congruence formula
        uint32_t part1 = day;
        uint32_t part2 = ((month + 1) * 26) / 10;
        uint32_t part3 = year;
        uint32_t part4 = year / 4;
        uint32_t part5 = 6 * (year / 100);
        uint32_t part6 = year / 400;

        uint32_t val = (part1 + part2 + part3 + part4 + part5 + part6) % 7;

        if (val == 0) {
            val = 7;
        }
        return val - 1;
    }

    /**
    * Get the next month based on the current month.
    *
    * @param curMonth The current month (1 - 12).
    * @return The next month (1 - 12). If the current month is 12, it returns 1.
    */
    uint8_t getNextMonth(uint8_t curMonth)
    {
        return ((curMonth < 12u) ? (curMonth + 1u) : 1u);
    }

    /**
    * Get the next year based on the current year.
    *
    * @param curYear The current year.
    * @return The next year.
    */
    uint16_t getNextYear(uint16_t curYear)
    {
        return (curYear + 1u);
    }

    /**
     * Determine whether a given year is a leap year.
     *
     * @param year The year to check.
     * @return 1 if the year is a leap year, 0 otherwise.
     */
    uint32_t getLeapYear(uint32_t year)
    {
        uint32_t isDivisibleBy4 = (year % 4 == 0);
        uint32_t isDivisibleBy100 = (year % 100 == 0);
        uint32_t isDivisibleBy400 = (year % 400 == 0);

        return (isDivisibleBy4 && !isDivisibleBy100) || isDivisibleBy400;
    }

    /**
    * Get the number of days in a given month of a specific year, considering leap years.
    *
    * @param month The month (1 - 12).
    * @param year The year.
    * @return The number of days in the specified month of the given year.
    */
    uint8_t getDaysInMonth(uint8_t month, uint16_t year)
    {
        constexpr uint8_t DAYS_IN_JANUARY      = (31u);
        constexpr uint8_t DAYS_IN_FEBRUARY     = (28u);
        constexpr uint8_t DAYS_IN_MARCH        = (31u);
        constexpr uint8_t DAYS_IN_APRIL        = (30u);
        constexpr uint8_t DAYS_IN_MAY          = (31u);
        constexpr uint8_t DAYS_IN_JUNE         = (30u);
        constexpr uint8_t DAYS_IN_JULY         = (31u);
        constexpr uint8_t DAYS_IN_AUGUST       = (31u);
        constexpr uint8_t DAYS_IN_SEPTEMBER    = (30u);
        constexpr uint8_t DAYS_IN_OCTOBER      = (31u);
        constexpr uint8_t DAYS_IN_NOVEMBER     = (30u);
        constexpr uint8_t DAYS_IN_DECEMBER     = (31u);

        const uint8_t daysInMonthTable[12] = {DAYS_IN_JANUARY,
                                              DAYS_IN_FEBRUARY,
                                              DAYS_IN_MARCH,
                                              DAYS_IN_APRIL,
                                              DAYS_IN_MAY,
                                              DAYS_IN_JUNE,
                                              DAYS_IN_JULY,
                                              DAYS_IN_AUGUST,
                                              DAYS_IN_SEPTEMBER,
                                              DAYS_IN_OCTOBER,
                                              DAYS_IN_NOVEMBER,
                                              DAYS_IN_DECEMBER
                                             };

        uint8_t val = daysInMonthTable[month - 1u];
        if (2 == month) {
            if (0u != getLeapYear(year)) {
                val++;
            }
        }
        return val;
    }

    /**
    * Convert UTC time represented by individual values to the time in the specified timezone.
    *
    * @param year: The year of the UTC time.
    * @param month: The month of the UTC time (1 - 12).
    * @param day: The day of the UTC time.
    * @param hour: The hour of the UTC time (0 - 23).
    * @param minute: The minute of the UTC time (0 - 59).
    * @param second: The second of the UTC time (0 - 59).
    * @param timezoneOffsetSeconds: The offset of the target timezone from UTC (in seconds).
    */
    void convertUtcToTimezone(int &year, int &month, int &day, int &hour, int &minute, int &second, int timezoneOffsetSeconds)
    {
        if (year < 0 || month < 1 || month > 12 || day < 1 || day > getDaysInMonth(month, year) ||
                hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 59) {
            log_e("Invalid date or time input");
            return;
        }

        int totalSeconds = hour * 3600 + minute * 60 + second;
        totalSeconds += timezoneOffsetSeconds;

        int daysOffset = totalSeconds / (24 * 3600);
        totalSeconds %= (24 * 3600);

        adjustDate(year, month, day, daysOffset);

        if (totalSeconds < 0) {
            totalSeconds += 24 * 3600;
            adjustDate(year, month, day, -1);
        }

        hour = totalSeconds / 3600;
        minute = (totalSeconds % 3600) / 60;
        second = totalSeconds % 60;
    }

    /**
     * Overloaded function to convert UTC time represented by a struct tm object to the time in the specified timezone.
     *
     * @param utcTime: A reference to the struct tm object representing UTC time.
     * @param timezoneOffsetSeconds: The offset of the target timezone from UTC (in seconds).
     */
    void convertUtcToTimezone(struct tm& utcTime, int timezoneOffsetSeconds)
    {
        int year = utcTime.tm_year + 1900;
        int month = utcTime.tm_mon + 1;
        int day = utcTime.tm_mday;
        int hour = utcTime.tm_hour;
        int minute = utcTime.tm_min;
        int second = utcTime.tm_sec;

        convertUtcToTimezone(year, month, day, hour, minute, second, timezoneOffsetSeconds);

        utcTime.tm_year = year - 1900;
        utcTime.tm_mon = month - 1;
        utcTime.tm_mday = day;
        utcTime.tm_hour = hour;
        utcTime.tm_min = minute;
        utcTime.tm_sec = second;
    }

    /**
     * Convert a Binary Coded Decimal (BCD) number to a decimal number.
     * @param val A uint8_t value representing a BCD number.
     * @return The decimal equivalent of the input BCD number.
     */
    uint8_t BCD2DEC(uint8_t val)
    {
        return ((val >> 4) * 10) + (val & 0x0F);
    }

    /**
     * Convert a decimal number to a Binary Coded Decimal (BCD) number.
     * @param val A uint8_t value representing a decimal number.
     * @return The BCD equivalent of the input decimal number.
     */
    uint8_t DEC2BCD(uint8_t val)
    {
        return ((val / 10) << 4) | (val % 10);
    }

private:

    static const size_t FORMAT_BUFFER_SIZE = 64;


    void adjustDate(int &year, int &month, int &day, int daysOffset)
    {
        day += daysOffset;
        while (day > getDaysInMonth(month, year)) {
            day -= getDaysInMonth(month, year);
            month++;
            if (month > 12) {
                year++;
                month = 1;
            }
        }
        while (day < 1) {
            month--;
            if (month < 1) {
                year--;
                month = 12;
            }
            day += getDaysInMonth(month, year);
        }
    }

    int formatDateTime(const char *fmt, char *buffer, size_t bufferSize, const RTC_DateTime *t)
    {
        const char *weekString[] = {"Sun", "Mon", "Tue", "Wed", "Thur", "Fri", "Sat"};
        if (!fmt || !buffer || bufferSize == 0 || !t) {
            return -1;
        }
        return snprintf(buffer, bufferSize, fmt, t->getYear(), t->getMonth(), t->getDay(),
                        t->getHour(), t->getMinute(), t->getSecond(),
                        t->getWeek() > 6 ? weekString[0] : weekString[t->getWeek()]);
    }
};
