#pragma once

#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace acs::config {

// Lightweight JSON value class for configuration
class JsonValue {
public:
  enum class Type { Null, Bool, Number, String, Array, Object };

  JsonValue() : type_(Type::Null) {}
  explicit JsonValue(bool b) : type_(Type::Bool), bool_val_(b) {}
  explicit JsonValue(double n) : type_(Type::Number), number_val_(n) {}
  explicit JsonValue(const std::string& s) : type_(Type::String), string_val_(s) {}

  // Copy constructor
  JsonValue(const JsonValue& other) : type_(other.type_), bool_val_(other.bool_val_),
                                       number_val_(other.number_val_), string_val_(other.string_val_) {
    if (other.array_val_) array_val_ = std::make_unique<std::vector<JsonValue>>(*other.array_val_);
    if (other.object_val_) object_val_ = std::make_unique<std::map<std::string, JsonValue>>(*other.object_val_);
  }

  // Copy assignment
  JsonValue& operator=(const JsonValue& other) {
    if (this != &other) {
      type_ = other.type_;
      bool_val_ = other.bool_val_;
      number_val_ = other.number_val_;
      string_val_ = other.string_val_;
      if (other.array_val_) array_val_ = std::make_unique<std::vector<JsonValue>>(*other.array_val_);
      else array_val_.reset();
      if (other.object_val_) object_val_ = std::make_unique<std::map<std::string, JsonValue>>(*other.object_val_);
      else object_val_.reset();
    }
    return *this;
  }

  // Move constructor and assignment
  JsonValue(JsonValue&&) = default;
  JsonValue& operator=(JsonValue&&) = default;

  ~JsonValue() = default;

  Type type() const { return type_; }
  bool is_null() const { return type_ == Type::Null; }
  bool is_bool() const { return type_ == Type::Bool; }
  bool is_number() const { return type_ == Type::Number; }
  bool is_string() const { return type_ == Type::String; }
  bool is_array() const { return type_ == Type::Array; }
  bool is_object() const { return type_ == Type::Object; }

  bool as_bool() const {
    if (type_ != Type::Bool) throw std::runtime_error("Not a boolean");
    return bool_val_;
  }

  double as_number() const {
    if (type_ != Type::Number) throw std::runtime_error("Not a number");
    return number_val_;
  }

  const std::string& as_string() const {
    if (type_ != Type::String) throw std::runtime_error("Not a string");
    return string_val_;
  }

  const std::vector<JsonValue>& as_array() const {
    if (type_ != Type::Array || !array_val_) throw std::runtime_error("Not an array");
    return *array_val_;
  }

  const std::map<std::string, JsonValue>& as_object() const {
    if (type_ != Type::Object || !object_val_) throw std::runtime_error("Not an object");
    return *object_val_;
  }

  // Helper to get object member safely
  JsonValue get(const std::string& key) const {
    if (type_ != Type::Object || !object_val_) return JsonValue();
    auto it = object_val_->find(key);
    return (it != object_val_->end()) ? it->second : JsonValue();
  }

  bool has(const std::string& key) const {
    return type_ == Type::Object && object_val_ && object_val_->count(key) > 0;
  }

  // Builder methods
  static JsonValue make_array(const std::vector<JsonValue>& arr) {
    JsonValue v;
    v.type_ = Type::Array;
    v.array_val_ = std::make_unique<std::vector<JsonValue>>(arr);
    return v;
  }

  static JsonValue make_object(const std::map<std::string, JsonValue>& obj) {
    JsonValue v;
    v.type_ = Type::Object;
    v.object_val_ = std::make_unique<std::map<std::string, JsonValue>>(obj);
    return v;
  }

private:
  Type type_{Type::Null};
  bool bool_val_{false};
  double number_val_{0.0};
  std::string string_val_;
  std::unique_ptr<std::vector<JsonValue>> array_val_;
  std::unique_ptr<std::map<std::string, JsonValue>> object_val_;
};

// Simple JSON parser
class JsonParser {
public:
  static JsonValue parse(const std::string& json) {
    JsonParser parser(json);
    return parser.parse_value();
  }

  static JsonValue parse_file(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
      throw std::runtime_error("Failed to open file: " + path);
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return parse(buffer.str());
  }

private:
  explicit JsonParser(const std::string& json) : json_(json), pos_(0) {}

  char peek() const {
    skip_whitespace();
    return pos_ < json_.size() ? json_[pos_] : '\0';
  }

  char consume() {
    skip_whitespace();
    return pos_ < json_.size() ? json_[pos_++] : '\0';
  }

  void skip_whitespace() const {
    while (pos_ < json_.size() &&
           (json_[pos_] == ' ' || json_[pos_] == '\t' ||
            json_[pos_] == '\n' || json_[pos_] == '\r')) {
      ++pos_;
    }
  }

  JsonValue parse_value() {
    char c = peek();
    if (c == '{') return parse_object();
    if (c == '[') return parse_array();
    if (c == '"') return parse_string();
    if (c == 't' || c == 'f') return parse_bool();
    if (c == 'n') return parse_null();
    if (c == '-' || (c >= '0' && c <= '9')) return parse_number();
    throw std::runtime_error("Unexpected character in JSON");
  }

  JsonValue parse_object() {
    consume(); // '{'
    std::map<std::string, JsonValue> obj;

    if (peek() == '}') {
      consume();
      return JsonValue::make_object(obj);
    }

    while (true) {
      std::string key = parse_string().as_string();
      if (consume() != ':') throw std::runtime_error("Expected ':'");
      JsonValue value = parse_value();
      obj[key] = value;

      char c = consume();
      if (c == '}') break;
      if (c != ',') throw std::runtime_error("Expected ',' or '}'");
    }

    return JsonValue::make_object(obj);
  }

  JsonValue parse_array() {
    consume(); // '['
    std::vector<JsonValue> arr;

    if (peek() == ']') {
      consume();
      return JsonValue::make_array(arr);
    }

    while (true) {
      arr.push_back(parse_value());
      char c = consume();
      if (c == ']') break;
      if (c != ',') throw std::runtime_error("Expected ',' or ']'");
    }

    return JsonValue::make_array(arr);
  }

  JsonValue parse_string() {
    consume(); // '"'
    std::string str;
    while (pos_ < json_.size() && json_[pos_] != '"') {
      if (json_[pos_] == '\\') {
        ++pos_;
        if (pos_ >= json_.size()) throw std::runtime_error("Unexpected end in string");
        char c = json_[pos_++];
        switch (c) {
          case '"': str += '"'; break;
          case '\\': str += '\\'; break;
          case '/': str += '/'; break;
          case 'n': str += '\n'; break;
          case 't': str += '\t'; break;
          default: str += c;
        }
      } else {
        str += json_[pos_++];
      }
    }
    if (pos_ >= json_.size()) throw std::runtime_error("Unterminated string");
    ++pos_; // closing '"'
    return JsonValue(str);
  }

  JsonValue parse_number() {
    size_t start = pos_;
    if (json_[pos_] == '-') ++pos_;
    while (pos_ < json_.size() && ((json_[pos_] >= '0' && json_[pos_] <= '9') ||
                                     json_[pos_] == '.' || json_[pos_] == 'e' ||
                                     json_[pos_] == 'E' || json_[pos_] == '+' ||
                                     json_[pos_] == '-')) {
      ++pos_;
    }
    std::string num_str = json_.substr(start, pos_ - start);
    return JsonValue(std::stod(num_str));
  }

  JsonValue parse_bool() {
    if (json_.substr(pos_, 4) == "true") {
      pos_ += 4;
      return JsonValue(true);
    }
    if (json_.substr(pos_, 5) == "false") {
      pos_ += 5;
      return JsonValue(false);
    }
    throw std::runtime_error("Expected boolean");
  }

  JsonValue parse_null() {
    if (json_.substr(pos_, 4) == "null") {
      pos_ += 4;
      return JsonValue();
    }
    throw std::runtime_error("Expected null");
  }

  std::string json_;
  mutable size_t pos_;
};

} // namespace acs::config
