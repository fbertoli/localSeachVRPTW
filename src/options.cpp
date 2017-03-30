#include <string>
#include <iostream>
#include <algorithm>

#include "options.h"

using namespace std;

Options* Options::_inst = nullptr;

bool Options::_parsed = false;

bool Options::_help_requested = false;

string Options::_prefix = "--";

BaseOption::BaseOption(const string& name, const string& description) :
  _name(name),
  _description(description),
  _set(false)
{
  Options::get().add(this);
}

BaseOption::~BaseOption()
{
  Options::get().remove(this);
}

ostream& operator<< (ostream& os, const BaseOption& o)
{
  return o.print(os);
}

Options& Options::get()
{
  if (_inst == nullptr)
    _inst = new Options();
  return *_inst;
}

void Options::remove(BaseOption *o)
{
  auto os = _opts[o->_name];
  auto p = std::find(os.begin(), os.end(), o);
  if (p != os.end())
    os.erase(p);
}

void Options::help()
{
  cerr << Options::get() << endl;
}

void Options::parse(int argc, char** argv)
{
  if (_parsed)
    return;

  Options& o = Options::get();

  // store name of the executable (index 0)
  o._executable = argv[0];

  // then start from second index
  int p_index = 1;
  while (p_index < argc)
  {
    // get key, value
    auto key = string(argv[p_index++]);

    // find whether the prefix is there
    auto index = key.find(_prefix, 0);

    if (index != 0 || index == string::npos)
      throw runtime_error("Parameter " + key + " does not start with prefix (" + _prefix + ")");

    key = key.replace(index, _prefix.length(), "");

    if (key == "help")
    {
      _help_requested = true;
      continue;
    }

    auto value = argv[p_index++];

    o._vals[key] = value;
  }

  // assign parsed values to already declared parameters
  for (const auto& key_value : o._vals)
  {
    auto optsi = o._opts.find(key_value.first);
    if (optsi != o._opts.end())
      for (auto& o : optsi->second)
        o->parse(key_value.second);
  }

  _parsed = true;
}

void Options::help_if_requested()
{
  if (_help_requested)
  {
    help();
    std::exit(0);
  }
}

void Options::set_prefix(const string& p)
{
  Options::_prefix = p;
}

void Options::add(BaseOption* o)
{
  _opts[o->_name].push_back(o);
}

void Options::check_value(BaseOption *o)
{
  if (_vals.find(o->_name) != _vals.end())
    o->parse(_vals[o->_name]);
}

ostream& operator<<(ostream& os, const Options& o)
{
  if (o._opts.empty())
    return os;

  os << "Usage: " << o._executable << " OPTIONS" << endl;
  os << "Options: " << endl << endl;
  for (auto& opts : o._opts)
  {
    auto& op = opts.second.front();
    os << "  " << Options::_prefix << op->_name << " ";
    op->print(os, true);
    os << ", " << op->_description << endl;
  }

  return os;
}
