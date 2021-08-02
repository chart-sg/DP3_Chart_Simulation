#include <iostream>
#include <thread>
// #include <memory>
#include <fstream>
#include <vector>

class MoveTo
{
    // public: MoveTo()
    // {
    //     // Read from the text file
    //     std::ifstream MyReadFile("location.txt", std::ios::in);

    //     //check to see that the file was opened correctly:
    //     if (!MyReadFile.is_open()) {
    //         std::cerr << "There was a problem opening the input file!\n";
    //         exit(1);//exit or do additional error checking
    //     }

    //     double num = 0.0;
    //     //keep storing values from the text file so long as data exists:
    //     while (MyReadFile >> num) {
    //         _location.push_back(num);
    //     }

    //     for (const auto& val : _location)
    //         std::cout<<" "<<val<<" "<<std::endl;

    //     // Close the file
    //     MyReadFile.close(); 
    // }

    public: void check_input(double* location)
    {
        std::cout<<
            "Input the coordinates you want to move the model." 
            "Type in \"quit\" to quit"
        <<std::endl;
        std::cin>>x>>y>>z;
        // std::cin>>y;
        // std::cin>>z;
        location[0]= x;//_location[0];
        location[1]= y;//_location[1];
        location[2]= z;//_location[2];
        std::cout<<"Movng model to positions" 
            <<" X: "<<x
            <<" Y: "<<y
            <<" Z: "<<z<<std::endl;
    }
    private: std::string myText;
    private: std::vector<double> _location;    
    private: double x = 0.0, y=0.0, z=0.0;
};

int main(int argc, char** argv)
{
    MoveTo mt;
}