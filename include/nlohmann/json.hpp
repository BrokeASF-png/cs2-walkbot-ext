#pragma once
#include <string>
#include <vector>
#include <map>
#include <stdexcept>
#include <sstream>
#include <fstream>
#include <cctype>
#include <functional>

namespace nlohmann
{
    class json
    {
    public:
        enum class value_t { null, object, array, string, number, boolean };

        json() : type_(value_t::null) {}
        json(const std::string& s) : type_(value_t::string), string_value(s) {}
        json(const char* s) : type_(value_t::string), string_value(s) {}
        json(float f) : type_(value_t::number), number_value(f) {}
        json(double d) : type_(value_t::number), number_value(d) {}
        json(int i) : type_(value_t::number), number_value(i) {}
        json(bool b) : type_(value_t::boolean), bool_value(b) {}

        json& operator[](const std::string& key)
        {
            if (type_ == value_t::null) type_ = value_t::object;
            return object_value[key];
        }

        const json& operator[](const std::string& key) const
        {
            static json null_json;
            auto it = object_value.find(key);
            return it != object_value.end() ? it->second : null_json;
        }

        json& operator[](size_t idx)
        {
            if (type_ == value_t::null) type_ = value_t::array;
            if (idx >= array_value.size()) array_value.resize(idx + 1);
            return array_value[idx];
        }

        const json& operator[](size_t idx) const
        {
            static json null_json;
            if (idx < array_value.size()) return array_value[idx];
            return null_json;
        }

        bool contains(const std::string& key) const
        {
            return type_ == value_t::object && object_value.count(key) > 0;
        }

        bool is_array() const { return type_ == value_t::array; }
        bool is_object() const { return type_ == value_t::object; }
        bool is_string() const { return type_ == value_t::string; }
        bool is_number() const { return type_ == value_t::number; }

        template<typename T> T get() const;

        static json parse(std::ifstream& file)
        {
            std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
            return parse_string(content);
        }

        static json parse(const std::string& str)
        {
            return parse_string(str);
        }

        static json array()
        {
            json j;
            j.type_ = value_t::array;
            return j;
        }

        void push_back(const json& j)
        {
            if (type_ == value_t::null) type_ = value_t::array;
            array_value.push_back(j);
        }

        std::string dump(int indent = -1) const
        {
            return dump_impl(indent >= 0, 0, indent);
        }

        auto begin() { return array_value.begin(); }
        auto end() { return array_value.end(); }

    private:
        value_t type_;
        std::map<std::string, json> object_value;
        std::vector<json> array_value;
        std::string string_value;
        double number_value = 0;
        bool bool_value = false;

        std::string dump_impl(bool pretty, int depth, int indent_size) const
        {
            std::stringstream ss;

            switch (type_)
            {
            case value_t::null:
                ss << "null";
                break;
            case value_t::boolean:
                ss << (bool_value ? "true" : "false");
                break;
            case value_t::number:
                ss << number_value;
                break;
            case value_t::string:
                ss << "\"" << string_value << "\"";
                break;
            case value_t::array:
                ss << "[";
                if (pretty && indent_size >= 0) ss << "\n";
                for (size_t i = 0; i < array_value.size(); ++i)
                {
                    if (pretty && indent_size >= 0)
                        ss << std::string((depth + 1) * indent_size, ' ');
                    ss << array_value[i].dump_impl(pretty, depth + 1, indent_size);
                    if (i < array_value.size() - 1) ss << ",";
                    if (pretty && indent_size >= 0) ss << "\n";
                }
                if (pretty && indent_size >= 0)
                    ss << std::string(depth * indent_size, ' ');
                ss << "]";
                break;
            case value_t::object:
                ss << "{";
                if (pretty && indent_size >= 0) ss << "\n";
                size_t i = 0;
                for (const auto& kv : object_value)
                {
                    if (pretty && indent_size >= 0)
                        ss << std::string((depth + 1) * indent_size, ' ');
                    ss << "\"" << kv.first << "\":";
                    if (pretty && indent_size >= 0) ss << " ";
                    ss << kv.second.dump_impl(pretty, depth + 1, indent_size);
                    if (++i < object_value.size()) ss << ",";
                    if (pretty && indent_size >= 0) ss << "\n";
                }
                if (pretty && indent_size >= 0)
                    ss << std::string(depth * indent_size, ' ');
                ss << "}";
                break;
            }

            return ss.str();
        }

        static json parse_string(const std::string& str)
        {
            json result;
            size_t pos = 0;

            auto skip_whitespace = [&]() {
                while (pos < str.length() && std::isspace((unsigned char)str[pos])) ++pos;
            };

            std::function<json()> parse_value = [&]() -> json {
                skip_whitespace();
                if (pos >= str.length()) return json();

                if (str[pos] == '{')
                {
                    json obj;
                    obj.type_ = value_t::object;
                    ++pos;
                    skip_whitespace();
                    while (pos < str.length() && str[pos] != '}')
                    {
                        skip_whitespace();
                        if (str[pos] != '"') return json();
                        ++pos;
                        std::string key;
                        while (pos < str.length() && str[pos] != '"')
                            key += str[pos++];
                        ++pos;
                        skip_whitespace();
                        if (pos >= str.length() || str[pos] != ':') return json();
                        ++pos;
                        obj[key] = parse_value();
                        skip_whitespace();
                        if (pos < str.length() && str[pos] == ',') ++pos;
                    }
                    if (pos < str.length()) ++pos;
                    return obj;
                }
                else if (str[pos] == '[')
                {
                    json arr;
                    arr.type_ = value_t::array;
                    ++pos;
                    skip_whitespace();
                    while (pos < str.length() && str[pos] != ']')
                    {
                        arr.array_value.push_back(parse_value());
                        skip_whitespace();
                        if (pos < str.length() && str[pos] == ',') ++pos;
                    }
                    if (pos < str.length()) ++pos;
                    return arr;
                }
                else if (str[pos] == '"')
                {
                    json s;
                    s.type_ = value_t::string;
                    ++pos;
                    while (pos < str.length() && str[pos] != '"')
                        s.string_value += str[pos++];
                    ++pos;
                    return s;
                }
                else if (std::isdigit((unsigned char)str[pos]) || str[pos] == '-')
                {
                    json num;
                    num.type_ = value_t::number;
                    std::string numStr;
                    if (str[pos] == '-') numStr += str[pos++];
                    while (pos < str.length() && (std::isdigit((unsigned char)str[pos]) || str[pos] == '.'))
                        numStr += str[pos++];
                    num.number_value = std::stod(numStr);
                    return num;
                }
                else if (str.compare(pos, 4, "true") == 0)
                {
                    json b;
                    b.type_ = value_t::boolean;
                    b.bool_value = true;
                    pos += 4;
                    return b;
                }
                else if (str.compare(pos, 5, "false") == 0)
                {
                    json b;
                    b.type_ = value_t::boolean;
                    b.bool_value = false;
                    pos += 5;
                    return b;
                }
                else if (str.compare(pos, 4, "null") == 0)
                {
                    pos += 4;
                    return json();
                }

                return json();
            };

            return parse_value();
        }
    };

    template<> inline float json::get<float>() const
    {
        if (type_ == value_t::number) return (float)number_value;
        throw std::runtime_error("json: type mismatch");
    }

    template<> inline double json::get<double>() const
    {
        if (type_ == value_t::number) return number_value;
        throw std::runtime_error("json: type mismatch");
    }

    template<> inline int json::get<int>() const
    {
        if (type_ == value_t::number) return (int)number_value;
        throw std::runtime_error("json: type mismatch");
    }

    template<> inline std::string json::get<std::string>() const
    {
        if (type_ == value_t::string) return string_value;
        throw std::runtime_error("json: type mismatch");
    }

    template<> inline bool json::get<bool>() const
    {
        if (type_ == value_t::boolean) return bool_value;
        throw std::runtime_error("json: type mismatch");
    }
}
