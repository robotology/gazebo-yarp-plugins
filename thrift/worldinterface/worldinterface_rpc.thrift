#world interface RPC server
namespace yarp GazeboYarpPlugins

service WorldInterfaceServer {
    string makeSphere (1:double x, 2:double y, 3:double z, 4:double radius, 5:byte r, 6:byte g, 7:byte b);
    string makeBox (1:double x, 2:double y, 3:double z, 4:double lx, 5:double ly, 6:double lz, 7:byte r, 8:byte g, 9:byte b);
    string makeCyl (1:double x, 2:double y, 3:double z, 4:double l, 5:double radius, 6:byte r, 7:byte g, 8:byte b);
    
    bool setPosition(1:string id, 2: double x, 3: double y, 4: double z);
    list<double> getPosition(1:string id);
        
    
    bool deleteAll();
    list<string> getList();
    
}
