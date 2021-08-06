#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include <iostream>
#include <thread>
#include <vector>

#include "include/rapidjson/document.h"
#include "include/rapidjson/writer.h"
#include "include/rapidjson/stringbuffer.h"

#include <string>
#include <fstream>
#include <streambuf>
#include <mutex>

/// \example examples/plugins/world_edit.cc
/// This example creates a WorldPlugin, initializes the Transport system by
/// creating a new Node, and publishes messages to alter gravity.
namespace gazebo
{
  class WorldEdit : public WorldPlugin
  {
    public: WorldEdit()
    {
        // Read the string
        std::ifstream t("/home/jh/Dev/dp3_chart_sim_ws"
                        "/install/tracking_database/share/"
                        "tracking_database/resource/output.json");
        std::string str;
        std::string location;
        std::array<int, 5> number_of_casualties = {-1, -1, -1, -1, -1};

        t.seekg(0, std::ios::end);   
        str.reserve(t.tellg());
        t.seekg(0, std::ios::beg);

        str.assign((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());

        // Parse the string to json
        rapidjson::Document document;
        document.Parse(str.c_str());

        // Parse json to get the location and the number_of_casualties
        assert(document.IsArray());
        for (rapidjson::Value::ConstValueIterator itr = document.Begin(); 
            itr != document.End(); 
            ++itr) 
        {
            const rapidjson::Value& _location = *itr;
            assert(_location.IsObject()); // each _location is an object
            for (rapidjson::Value::ConstMemberIterator itr2 = _location.MemberBegin(); 
                itr2 != _location.MemberEnd(); 
                ++itr2) 
            {
                if (itr2->value.IsNull())
                {
                    std::cout<<"Skipping NULL routes"<<std::endl;
                    continue;
                }
                if (itr2->name.GetString() == std::string("location"))
                {
                    location = itr2->value.GetString();
                }
                else if (itr2->value.IsObject())
                {
                    const auto& _casualties = itr2->value;
                    number_of_casualties[0] = _casualties["total"].GetInt(); 
                    number_of_casualties[1] = _casualties["P0"].GetInt();
                    number_of_casualties[2] = _casualties["P1"].GetInt();
                    number_of_casualties[3] = _casualties["P2"].GetInt();
                    number_of_casualties[4] = _casualties["P3"].GetInt();
                }
                else if (itr2->name.IsString())
                {
                    std::cout<<"Something unexpected happened!"<<std::endl;
                    break;
                }
            }
            if (number_of_casualties[0] != -1)
            {
                casualties.insert(std::make_pair(location, number_of_casualties));
            }
        }
        std::cout<<"Getting casualty numbers"<<std::endl;
        for (const auto& v : casualties)
        {
            std::cout<<"Location at "<<v.first<<" ";
            for (const auto& a : v.second)
            {
                std::cout<<a<<" ";
            }
            std::cout<<std::endl;
        }
        std::cout<<std::endl;
    }

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    { 
      const std::lock_guard<std::mutex> lock(mtx);
      int number_of_casualties = -1;

      for (int i = 0; i <casualties.at("CGH")[0]; i++)
      {
        sdf::SDF patientSDF;
        patientSDF.SetFromString(
          "<sdf version ='1.4'>\
            <model name ='sphere'>\
              <pose>15.0 -20.0 0.0 0 0 0.0</pose>\
              <link name ='link'>\
                <collision name ='collision'>\
                  <pose>0.0 0.0 1.0 0 0 0.0</pose>\
                  <geometry>\
                    <sphere><radius>0.5</radius></sphere>\
                  </geometry>\
                </collision>\
                <visual name ='visual'>\
                  <geometry>\
                    <mesh>\
                      <uri>model://MaleVisitorPhone/meshes/MaleVisitorPhoneWalk.dae</uri>\
                    </mesh>\
                  </geometry>\
                </visual>\
              </link>\
              <static>False</static>\
            </model>\
          </sdf>");
        // Demonstrate using a custom model name.
        sdf::ElementPtr model = patientSDF.Root()->GetElement("model");
        std::cout<<"I is now at "<<i<<std::endl;
        std::string model_name = "patient" + std::to_string(i);
        model->GetAttribute("name")->SetFromString(model_name);
        _parent->InsertModelSDF(patientSDF);
      }
    }

    public : std::unordered_map<std::string, std::array<int, 5>> casualties;
    public : std::unordered_map<std::string, std::array<int, 2>> location;
    private: std::mutex mtx; 
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldEdit)
}
