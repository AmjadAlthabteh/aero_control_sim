#pragma once

#include <iomanip>
#include <fstream>
#include <initializer_list>
#include <string>
#include <utility>

namespace aero
{
class CsvWriter
{
public:
    explicit CsvWriter(std::string path)
        : path_(std::move(path)), out_(path_, std::ios::out | std::ios::trunc)
    {
        out_ << std::fixed << std::setprecision(10);
    }

    bool ok() const { return out_.good(); }
    const std::string& path() const { return path_; }

    void writeHeader(std::initializer_list<std::string> cols)
    {
        bool first = true;
        for (const auto& c : cols)
        {
            if (!first)
                out_ << ',';
            first = false;
            out_ << c;
        }
        out_ << '\n';
    }

    void writeRow(std::initializer_list<double> values)
    {
        bool first = true;
        for (double v : values)
        {
            if (!first)
                out_ << ',';
            first = false;
            out_ << v;
        }
        out_ << '\n';
    }

private:
    std::string path_;
    std::ofstream out_;
};

} // namespace aero
