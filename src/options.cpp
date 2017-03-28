#include <string>
#include <iostream>

#include "options.h"

using namespace std;

Options* Options::_inst = nullptr;

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

const BaseOption& Options::operator[](const string& key) const
{
  auto oit = _opts.find(key);
  if (oit == _opts.end())
    throw std::runtime_error("Option " + key + " not found.");
  return *oit->second;
}

const BaseOption& Options::get(const string& key)
{
  return Options::get()[key];
}

void Options::help()
{
  cerr << Options::get();
}

void Options::parse(int argc, char** argv)
{
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
    auto index = 0u;
    index = key.find(_prefix, index);

    if (index != 0 || index == string::npos)
      throw runtime_error("Parameter " + key + " does not start with prefix (" + _prefix + ")");

    key = key.replace(index, _prefix.length(), "");

    if (key == "help")
    {
      Options::help();
      continue;
    }

    auto value = argv[p_index++];

    if (o._opts.find(key) != o._opts.end())
      o._opts[key]->parse(value);
  }
}

void Options::set_prefix(const string& p)
{
  Options::_prefix = p;
}

Options& Options::add(BaseOption* o)
{
  _opts[o->_name] = o;
  return *this;
}

Options& Options::remove(BaseOption* o)
{
  _opts.erase(_opts.find(o->_name));
  return *this;
}

ostream& operator<<(ostream& os, const Options& o)
{
  if (o._opts.empty())
    return os;

  os << "Usage: " << o._executable << " OPTIONS" << endl;
  os << "Options: " << endl << endl;
  for (auto& op : o._opts)
  {
    os << "  " << Options::_prefix << op.second->_name << " ";
    op.second->print(os, true);
    os << ", " << op.second->_description << endl;
  }

  return os;
}
