//#line 2 "/opt/ros/noetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"
// *********************************************************
//
// File autogenerated for the px4ctrl package
// by the dynamic_reconfigure package.
// Please do not edit.
//
// ********************************************************/

#ifndef __px4ctrl__FAKE_RCCONFIG_H__
#define __px4ctrl__FAKE_RCCONFIG_H__

#if __cplusplus >= 201103L
#define DYNAMIC_RECONFIGURE_FINAL final
#else
#define DYNAMIC_RECONFIGURE_FINAL
#endif

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace px4ctrl
{
  class fake_rcConfigStatics;

  class fake_rcConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l,
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }
      virtual ~AbstractParamDescription() = default;

      virtual void clamp(fake_rcConfig &config, const fake_rcConfig &max, const fake_rcConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const fake_rcConfig &config1, const fake_rcConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, fake_rcConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const fake_rcConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, fake_rcConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const fake_rcConfig &config) const = 0;
      virtual void getValue(const fake_rcConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template <class T>
    class ParamDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string a_name, std::string a_type, uint32_t a_level,
          std::string a_description, std::string a_edit_method, T fake_rcConfig::* a_f) :
        AbstractParamDescription(a_name, a_type, a_level, a_description, a_edit_method),
        field(a_f)
      {}

      T fake_rcConfig::* field;

      virtual void clamp(fake_rcConfig &config, const fake_rcConfig &max, const fake_rcConfig &min) const override
      {
        if (config.*field > max.*field)
          config.*field = max.*field;

        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const fake_rcConfig &config1, const fake_rcConfig &config2) const override
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, fake_rcConfig &config) const override
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const fake_rcConfig &config) const override
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, fake_rcConfig &config) const override
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const fake_rcConfig &config) const override
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const fake_rcConfig &config, boost::any &val) const override
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      virtual ~AbstractGroupDescription() = default;

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, fake_rcConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template<class T, class PT>
    class GroupDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string a_name, std::string a_type, int a_parent, int a_id, bool a_s, T PT::* a_f) : AbstractGroupDescription(a_name, a_type, a_parent, a_id, a_s), field(a_f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const override
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const override
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, fake_rcConfig &top) const override
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const override
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T PT::* field;
      std::vector<fake_rcConfig::AbstractGroupDescriptionConstPtr> groups;
    };

class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(fake_rcConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("cmd_int"==(*_i)->name){cmd_int = boost::any_cast<int>(val);}
        if("mode_int"==(*_i)->name){mode_int = boost::any_cast<int>(val);}
        if("cmd_bool"==(*_i)->name){cmd_bool = boost::any_cast<bool>(val);}
        if("mode_bool"==(*_i)->name){mode_bool = boost::any_cast<bool>(val);}
        if("size"==(*_i)->name){size = boost::any_cast<int>(val);}
      }
    }

    int cmd_int;
int mode_int;
bool cmd_bool;
bool mode_bool;
int size;

    bool state;
    std::string name;

    
}groups;



//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      int cmd_int;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      int mode_int;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      bool cmd_bool;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      bool mode_bool;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      int size;
//#line 231 "/opt/ros/noetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("fake_rcConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }

    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }

    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const fake_rcConfig &__max__ = __getMax__();
      const fake_rcConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const fake_rcConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }

    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const fake_rcConfig &__getDefault__();
    static const fake_rcConfig &__getMax__();
    static const fake_rcConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();

  private:
    static const fake_rcConfigStatics *__get_statics__();
  };

  template <> // Max and min are ignored for strings.
  inline void fake_rcConfig::ParamDescription<std::string>::clamp(fake_rcConfig &config, const fake_rcConfig &max, const fake_rcConfig &min) const
  {
    (void) config;
    (void) min;
    (void) max;
    return;
  }

  class fake_rcConfigStatics
  {
    friend class fake_rcConfig;

    fake_rcConfigStatics()
    {
fake_rcConfig::GroupDescription<fake_rcConfig::DEFAULT, fake_rcConfig> Default("Default", "", 0, 0, true, &fake_rcConfig::groups);
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.cmd_int = 0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.cmd_int = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.cmd_int = 0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(fake_rcConfig::AbstractParamDescriptionConstPtr(new fake_rcConfig::ParamDescription<int>("cmd_int", "int", 0, "An Integer parameter", "", &fake_rcConfig::cmd_int)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(fake_rcConfig::AbstractParamDescriptionConstPtr(new fake_rcConfig::ParamDescription<int>("cmd_int", "int", 0, "An Integer parameter", "", &fake_rcConfig::cmd_int)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.mode_int = 0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.mode_int = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.mode_int = 0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(fake_rcConfig::AbstractParamDescriptionConstPtr(new fake_rcConfig::ParamDescription<int>("mode_int", "int", 0, "An Integer parameter", "", &fake_rcConfig::mode_int)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(fake_rcConfig::AbstractParamDescriptionConstPtr(new fake_rcConfig::ParamDescription<int>("mode_int", "int", 0, "An Integer parameter", "", &fake_rcConfig::mode_int)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.cmd_bool = 0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.cmd_bool = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.cmd_bool = 0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(fake_rcConfig::AbstractParamDescriptionConstPtr(new fake_rcConfig::ParamDescription<bool>("cmd_bool", "bool", 0, "A Boolean parameter", "", &fake_rcConfig::cmd_bool)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(fake_rcConfig::AbstractParamDescriptionConstPtr(new fake_rcConfig::ParamDescription<bool>("cmd_bool", "bool", 0, "A Boolean parameter", "", &fake_rcConfig::cmd_bool)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.mode_bool = 0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.mode_bool = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.mode_bool = 0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(fake_rcConfig::AbstractParamDescriptionConstPtr(new fake_rcConfig::ParamDescription<bool>("mode_bool", "bool", 0, "A Boolean parameter", "", &fake_rcConfig::mode_bool)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(fake_rcConfig::AbstractParamDescriptionConstPtr(new fake_rcConfig::ParamDescription<bool>("mode_bool", "bool", 0, "A Boolean parameter", "", &fake_rcConfig::mode_bool)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.size = 0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.size = 3;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.size = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(fake_rcConfig::AbstractParamDescriptionConstPtr(new fake_rcConfig::ParamDescription<int>("size", "int", 0, "A size parameter which is edited via an enum", "{'enum': [{'name': 'Small', 'type': 'int', 'value': 0, 'srcline': 13, 'srcfile': '/home/mm/catkin_ws/src/px4ctrl/cfg/fake_rc.cfg', 'description': 'A small constant', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Medium', 'type': 'int', 'value': 1, 'srcline': 14, 'srcfile': '/home/mm/catkin_ws/src/px4ctrl/cfg/fake_rc.cfg', 'description': 'A medium constant', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Large', 'type': 'int', 'value': 2, 'srcline': 15, 'srcfile': '/home/mm/catkin_ws/src/px4ctrl/cfg/fake_rc.cfg', 'description': 'A large constant', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'ExtraLarge', 'type': 'int', 'value': 3, 'srcline': 16, 'srcfile': '/home/mm/catkin_ws/src/px4ctrl/cfg/fake_rc.cfg', 'description': 'An extra large constant', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'An enum to set size'}", &fake_rcConfig::size)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(fake_rcConfig::AbstractParamDescriptionConstPtr(new fake_rcConfig::ParamDescription<int>("size", "int", 0, "A size parameter which is edited via an enum", "{'enum': [{'name': 'Small', 'type': 'int', 'value': 0, 'srcline': 13, 'srcfile': '/home/mm/catkin_ws/src/px4ctrl/cfg/fake_rc.cfg', 'description': 'A small constant', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Medium', 'type': 'int', 'value': 1, 'srcline': 14, 'srcfile': '/home/mm/catkin_ws/src/px4ctrl/cfg/fake_rc.cfg', 'description': 'A medium constant', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Large', 'type': 'int', 'value': 2, 'srcline': 15, 'srcfile': '/home/mm/catkin_ws/src/px4ctrl/cfg/fake_rc.cfg', 'description': 'A large constant', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'ExtraLarge', 'type': 'int', 'value': 3, 'srcline': 16, 'srcfile': '/home/mm/catkin_ws/src/px4ctrl/cfg/fake_rc.cfg', 'description': 'An extra large constant', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'An enum to set size'}", &fake_rcConfig::size)));
//#line 246 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.convertParams();
//#line 246 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __group_descriptions__.push_back(fake_rcConfig::AbstractGroupDescriptionConstPtr(new fake_rcConfig::GroupDescription<fake_rcConfig::DEFAULT, fake_rcConfig>(Default)));
//#line 369 "/opt/ros/noetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

      for (std::vector<fake_rcConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__);
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__);
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__);
    }
    std::vector<fake_rcConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<fake_rcConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    fake_rcConfig __max__;
    fake_rcConfig __min__;
    fake_rcConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const fake_rcConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static fake_rcConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &fake_rcConfig::__getDescriptionMessage__()
  {
    return __get_statics__()->__description_message__;
  }

  inline const fake_rcConfig &fake_rcConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }

  inline const fake_rcConfig &fake_rcConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }

  inline const fake_rcConfig &fake_rcConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }

  inline const std::vector<fake_rcConfig::AbstractParamDescriptionConstPtr> &fake_rcConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<fake_rcConfig::AbstractGroupDescriptionConstPtr> &fake_rcConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const fake_rcConfigStatics *fake_rcConfig::__get_statics__()
  {
    const static fake_rcConfigStatics *statics;

    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = fake_rcConfigStatics::get_instance();

    return statics;
  }

//#line 13 "/home/mm/catkin_ws/src/px4ctrl/cfg/fake_rc.cfg"
      const int fake_rc_Small = 0;
//#line 14 "/home/mm/catkin_ws/src/px4ctrl/cfg/fake_rc.cfg"
      const int fake_rc_Medium = 1;
//#line 15 "/home/mm/catkin_ws/src/px4ctrl/cfg/fake_rc.cfg"
      const int fake_rc_Large = 2;
//#line 16 "/home/mm/catkin_ws/src/px4ctrl/cfg/fake_rc.cfg"
      const int fake_rc_ExtraLarge = 3;
}

#undef DYNAMIC_RECONFIGURE_FINAL

#endif // __FAKE_RCRECONFIGURATOR_H__
