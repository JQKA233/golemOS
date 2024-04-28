#pragma once
#include "frc/Errors.h"
#include "ctre/phoenix/ErrorCode.h"
#include <string>



class TalonUtil {
public:
    /**
    * 检查指定的错误代码是否存在问题
    *
    * @param errorCode 错误代码
    * @param message   如果有错误，打印出信息
    */
    void checkError(ctre::phoenix::ErrorCode errorCode,std::string message) {


        if (errorCode != ctre::phoenix::ErrorCode::OK) {

            const char* fileName = __FILE__; // 当前源文件名
            int lineNumber = __LINE__; // 当前行号
            const char* funcName = __func__; // 当前函数名

            const char* errormessage = message.c_str();
            frc::ReportError(static_cast<int32_t>(errorCode), fileName, lineNumber, funcName, errormessage);
        }
    }
};
