#pragma once
#include <iostream>
#include <sstream>
#include <cassert>
#include <vector>
#include <thread>
#include <chrono>

namespace Manifest_Utility
{
    using CONSOLE_CODE = const char*;

    //FORMAT
    static constexpr CONSOLE_CODE CONSOLE_RESET{ "0" };
    static constexpr CONSOLE_CODE CONSOLE_BOLD{ "1" };
    static constexpr CONSOLE_CODE CONSOLE_DIM{ "2" };
    static constexpr CONSOLE_CODE CONSOLE_ITALIC{ "3" };
    static constexpr CONSOLE_CODE CONSOLE_UNDERLINE{ "4" };
    static constexpr CONSOLE_CODE CONSOLE_BLINK{ "5" };
    static constexpr CONSOLE_CODE CONSOLE_REVERSE{ "7" };
    static constexpr CONSOLE_CODE CONSOLE_HIDDEN{ "8" };
    static constexpr CONSOLE_CODE CONSOLE_STRIKETHROUGH{ "9" };
    //FG COLOR
    static constexpr CONSOLE_CODE CONSOLE_BLACK{ "30" };
    static constexpr CONSOLE_CODE CONSOLE_RED{ "31" };
    static constexpr CONSOLE_CODE CONSOLE_GREEN{ "32" };
    static constexpr CONSOLE_CODE CONSOLE_YELLOW{ "33" };
    static constexpr CONSOLE_CODE CONSOLE_BLUE{ "34" };
    static constexpr CONSOLE_CODE CONSOLE_MAGENTA{ "35" };
    static constexpr CONSOLE_CODE CONSOLE_CYAN{ "36" };
    static constexpr CONSOLE_CODE CONSOLE_WHITE{ "37" };
    static constexpr CONSOLE_CODE CONSOLE_DEFAULT{ "39" };
    //BG COLOR
    static constexpr CONSOLE_CODE CONSOLE_BG_BLACK{ "40" };
    static constexpr CONSOLE_CODE CONSOLE_BG_RED{ "41" };
    static constexpr CONSOLE_CODE CONSOLE_BG_GREEN{ "42" };
    static constexpr CONSOLE_CODE CONSOLE_BG_YELLOW{ "43" };
    static constexpr CONSOLE_CODE CONSOLE_BG_BLUE{ "44" };
    static constexpr CONSOLE_CODE CONSOLE_BG_MAGENTA{ "45" };
    static constexpr CONSOLE_CODE CONSOLE_BG_CYAN{ "46" };
    static constexpr CONSOLE_CODE CONSOLE_BG_WHITE{ "47" };
    static constexpr CONSOLE_CODE CONSOLE_BG_DEFAULT{ "49" };


    template <typename T>
    void printCsv(std::ostream& os, T&& value) {
        os << std::forward<T>(value);
    }

    template <typename T, typename... Args>
    void printCsv(std::ostream& os, T&& value, Args&&... args) {
        os << std::forward<T>(value) << " ";
        printCsv(os, std::forward<Args>(args)...);
    }


    template <typename... Args>
    std::string LogConsole(std::vector<CONSOLE_CODE> consoleOptions, Args&&... args)
    {
        std::ostringstream result;
        result << "\x1B[";
        for (const CONSOLE_CODE& optionCode : consoleOptions)
            result << optionCode << ';';
        result.seekp(result.tellp() - std::streampos{ 1 });
        result << "m";
        //result << "THREAD: " << std::this_thread::get_id();
       // result << " TIME: " << std::chrono::system_clock::now() <<"\n";
        printCsv(result, std::forward<Args>(args)...);


        result << "\x1B[0m\n";
        return result.str();
    }

#define LOG(COLOR,...) std::cout << LogConsole(COLOR,__VA_ARGS__)

#ifndef _DEBUG //vs built in macro
#define DLOG(COLOR,x) 
#define RLOG(COLOR,...) LOG(COLOR,__VA_ARGS__)
#else
#define DLOG(COLOR,...) LOG(COLOR,__VA_ARGS__)
#define RLOG(COLOR,x)
#endif
}


#ifndef DISABLE
#define DISABLE if(0)//disable code path
#endif // !DISABLE

#ifndef DEFAULT_BREAK//used in switch statments to indicate defualt case may be used in the future 
#define DEFAULT_BREAK default:break;
#endif // !DEFAULT_BREAK
