#include <exceptions.h>

int AbstractException::get_error_number() const {
    return error_number_;
}

int AbstractException::get_line() const {
    return line_;
}

const std::string& AbstractException::get_file() const {
    return file_;
}

const std::string& AbstractException::get_func() const {
    return func_;
}

const std::string& AbstractException::get_info() const {
    return info_;
}

InputFileException::InputFileException(int error_number,
                                       int line,
                                       const std::string& file,
                                       const std::string& path,
                                       const std::string& func,
                                       const std::string& info) {
    error_number_ = error_number;
    line_ = line;
    file_ = file;
    path_ = path;
    func_ = func;
    info_ = info;
    error_msg.assign("Error " + std::to_string(error_number_) + " occured in the function " + func_ +
                     "\nSource code file: " + file + " on the line #" + std::to_string(line) +
                     "\nMesh file with the path " + path_ + " can not be loaded.\nAdditional information: " + info);
}

const std::string& InputFileException::get_path() const {
    return path_;
}

const char* InputFileException::what() const noexcept {
    return error_msg.c_str();
}

ComputationalException::ComputationalException(int error_number,
                                               int line,
                                               const std::string& file,
                                               const std::string& func,
                                               const std::string& info) {
    error_number_ = error_number;
    line_ = line;
    file_ = file;
    func_ = func;
    info_ = info;
    error_msg.assign("Error " + std::to_string(error_number_) + " occured in the function " + func_ +
                     "\nSource code file: " + file + " on the line #" + std::to_string(line) +
                     "\nAdditional information: " + info);
}

const char* ComputationalException::what() const noexcept {
    return error_msg.c_str();
}

GeneratorException::GeneratorException(int error_number,
                                       int line,
                                       const std::string& file,
                                       const std::string& step,
                                       const std::string& func,
                                       const std::string& info){
    error_number_ = error_number;
    line_ = line;
    file_ = file;
    step_ = step;
    func_ = func;
    info_ = info;
    error_msg.assign("Error " + std::to_string(error_number_) + " occured on the " + step_ + " step, in the function " + func_ +
                     "\nSource code file: " + file_ + " on the line #" + std::to_string(line) +
                     "\nAdditional information: " + info);
}

const char* GeneratorException::what() const noexcept {
    return error_msg.c_str();
}
