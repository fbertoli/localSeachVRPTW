#pragma once

#include <string>
#include <map>
#include <vector>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <typeinfo>
#include <type_traits>

/** Helper functions for parsing. */

template <typename T>
static typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, bool>::value, T>::type parse_t(const std::string& l)
{
  return std::atoll(l.c_str());
}

template <typename T>
static typename std::enable_if<std::is_floating_point<T>::value, T>::type parse_t(const std::string& l)
{
  return std::atof(l.c_str());
}

template <typename T>
static typename std::enable_if<std::is_same<T, bool>::value, T>::type parse_t(const std::string& l)
{
  if (l == "true" || l == "TRUE" || l == "True" || l == "T" || l == "1")
    return true;
  return false;
}

template <typename T>
static typename std::enable_if<std::is_same<T, std::string>::value, T>::type parse_t(const std::string& l)
{
  return l;
}

/** Helper functions for printing. */

template <typename T>
static std::ostream& print_t(const typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, bool>::value, T>::type& v, std::ostream& os = std::cout)
{
  return os << "[int, default: " << v << "]";
}

template <typename T>
static std::ostream& print_t(const typename std::enable_if<std::is_floating_point<T>::value, T>::type& v, std::ostream& os = std::cout)
{
  return os << "[float, default: " << v << "]";
}

template <typename T>
static std::ostream& print_t(const typename std::enable_if<std::is_same<T, std::string>::value, T>::type& v, std::ostream& os = std::cout)
{
  return os << "[string, default: " << v << "]";
}

template <typename T>
static std::ostream& print_t(const typename std::enable_if<std::is_same<T, bool>::value, T>::type& v, std::ostream& os = std::cout)
{
  return os << "[bool, default: " << v << "]";
}

class Options;

/** Abstract class for options, its main use being to arrange all the options
by name in a map (since maps don't allow heterogenous value types); methods
are dispatched later */
class BaseOption
{
public:

  /** So that only Options is able to call the parse methods of options */
  friend class Options;

  /** So that the Options output operators can access the print method */
  friend std::ostream& operator<< (std::ostream&, const Options&);

  /** So that the BaseOption output operators can access the print method */
  friend std::ostream& operator<< (std::ostream& os, const BaseOption& o);

  /** Print */
  virtual std::ostream& print(std::ostream& os = std::cout, bool detailed = false) const
  {
    return os;
  }

  /** Virtual parse method, to be specialized in template subclasses */
  virtual void parse(const std::string& l) = 0;

protected:

  /** Constructor, only meant to be called by a Option; also, registers the
  parameter in a global list of parameters (accessible by CLI) */
  BaseOption(const std::string& name, const std::string& description);

  /** Destructor, unregisters parameter from the global list of parameters
   *  (actually not anymore, becase it was a dumb thing to do, as hierarchy
   *  of classes owning parameters would cause the same parameter to be
   *  erased twice). Moreover, it is not particularly useful to unload
   *  parameters.
   */
  virtual ~BaseOption();

  /** Name of the option (for indexing) */
  std::string _name;

  /** Description of the option (for help) */
  std::string _description;

  /** A boolean which specifies is the value for this option is set */
  bool _set;

};

/** Template (generic) option class. */
template <typename T>
class Option : public BaseOption
{
public:

  using type = T;

  /** So that only Options is able to call the parse methods of options */
  friend class Options;

  /** So that the Options output operators can access the print method */
  friend std::ostream& operator<< (std::ostream&, const Options&);

  /** So that the BaseOption output operators can access the print method */
  friend std::ostream& operator<< (std::ostream& os, const BaseOption& o);

  /** Constructor, accepts a name and a default value */
  Option(
    const std::string& name,
    const std::string& description,
    const T& default_val
  );

  /** Automatic cast to base type */
  operator T() const
  {
    return _val;
  }

  /** Assignment operator */
  Option& operator=(const T& v)
  {
    _val = v;
    _set = true;
    return *this;
  }

  /** Parsing */
  virtual void parse(const std::string& l) override
  {
    this->_val = parse_t<T>(l);
    _set = true;
  }

  /** Print */
  std::ostream& print(std::ostream& os = std::cout, bool detailed = false) const override
  {
    if (detailed)
      return print_t<T>(this->_val, os);
    return os << this->_val;
  }

  bool is_set() const
  {
    return _set;
  }

  template <typename O = T>
  bool operator==(const O& o) const
  {
    return T(o) == _val;
  }

  operator const char* () const
  {
    std::stringstream ss;
    ss << this->_val;
    return ss.str().c_str();
  }

protected:

  /** Value of the parameter */
  T _val;
};

/** A singleton class representing the whole body of options */
class Options
{
public:

  /** So that the Options output operators can access the list of options */
  friend std::ostream& operator<< (std::ostream&, const Options&);

  /** Copy constructor (deleted) */
  Options(const Options&) = delete;

  /** Move constructor (deleted) */
  Options(Options&&) = delete;

  /** Assignment operator (deleted) */
  Options& operator=(const Options&) = delete;

  /** Move operator (deleted) */
  Options& operator=(Options&&) = delete;

  /** Get singleton instance */
  static Options& get();

  /** Overall parse */
  static void parse(int, char**);

  /** Register parameter */
  void add(BaseOption* o);

  /** Unregister parameter */
  void remove(BaseOption* o);

  /** Check if a value has been parsed. */
  void check_value(BaseOption* o);

  /** Set prefix of the options (default is --) */
  static void set_prefix(const std::string& s);

  /** Print help on stderr */
  static void help();

  /** Fetches an option by name. */
  const BaseOption& operator[](const std::string& key) const;

  /** Fetches an option by name (static). */
  static const BaseOption& get(const std::string& key);

  /** Checks whether a helper has been requested. */
  static void help_if_requested();

protected:

  /** Map of registered parameters (for printing instructions) */
  std::map<std::string, std::vector<BaseOption*>> _opts;

  /** Store values, so that we don't have to run parse multiple times. */
  std::map<std::string, std::string> _vals;

  /** Name of the executable. */
  std::string _executable;

  /** Pointer to the unique instance of Options */
  static Options* _inst;

  /** Prefix */
  static std::string _prefix;

  /** Help requested. */
  static bool _help_requested;

  static bool _parsed;

  /** Hidden default constructor */
  Options() = default;

};

template <typename T>
Option<T>::Option(const std::string &name, const std::string &description, const T &default_val) :
  BaseOption(name, description),
  _val(default_val)
{
  if (name == "help")
    throw std::runtime_error("Invalid name for option (reserved keyword): " + name);
  _set = false;
  Options::get().check_value(this);
}

/** Output operator for base option (for dispatching to specialized objects) */
std::ostream& operator<< (std::ostream& os, const BaseOption& o);

/** Overall output operator */
std::ostream& operator<< (std::ostream& os, const Options&);
