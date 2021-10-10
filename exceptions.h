#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

#include <exception>
#include <iostream>

class AbstractException : public std::exception
{
protected:
    int error_number_;
    int line_;
    std::string file_;
    std::string func_;
    std::string info_;
    std::string error_msg;

public:
    //explicit AbstractException(const std::string& msg, const std::string& file, int line, const std::string& func, const std::string& info = "");
    //virtual ~AbstractException() = 0;

    int get_error_number() const;
    int get_line() const;
    const std::string& get_file() const;
    const std::string& get_func() const;
    const std::string& get_info() const;

};

class InputFileException : public AbstractException
{
private:
    std::string path_;
public:
    explicit InputFileException(int error_number,
                                int line,
                                const std::string& file,
                                const std::string& path,
                                const std::string& func,
                                const std::string& info = "");

    const std::string& get_path() const;

    virtual const char* what() const noexcept override;
};

class ComputationalException : public AbstractException
{
public:
    explicit ComputationalException(int error_number,
                                    int line,
                                    const std::string& file,
                                    const std::string& func,
                                    const std::string& info = "");

    virtual const char* what() const noexcept override;
};

class GeneratorException : public AbstractException
{
private:
    std::string step_;
public:
    explicit GeneratorException(int error_number,
                                int line,
                                const std::string& file,
                                const std::string& step,
                                const std::string& func,
                                const std::string& info = "");

    virtual const char* what() const noexcept override;
};

#endif // EXCEPTIONS_H
