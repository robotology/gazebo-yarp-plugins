#include <multiexternalwrenchinterface.h>

using namespace gazebo;

MultiExternalWrenchInterface::MultiExternalWrenchInterface(): ModelPlugin(), m_network(0), m_rpcport(0)
{
}

MultiExternalWrenchInterface::~MultiExternalWrenchInterface()
{
    if(m_rpcport)
    {
        m_rpcport->close();
        delete m_rpcport;
    }
    
    if(m_network)
        delete m_network;
}

void MultiExternalWrenchInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if(m_network != 0)
        return;
    
    m_network = new yarp::os::Network();
    
    if(!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
    {
        yError() << "MultiExternalWrenchInterface::Load error: yarp network does not seem to be available, is the yarpserver running?";
        return;
    }
    
    bool configuration_loaded = false;
    
    if(_sdf->HasElement("robotNamefromConfigFile"))
    {
        std::string ini_file_name = _sdf->Get<std::string>("robotNamefromConfigFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);
        
        if(ini_file_path != "" && m_parameters->fromConfigFile(ini_file_path.c_str()))
        {
            yInfo() << "Found yarpConfigurationFile: loading from " << ini_file_path;
            configuration_loaded = true;
        }

        if(!configuration_loaded)
        {
            yError() << "ObjectAttacher: Load error, could not load configuration file";
            return;
        }

        //TODO Extend this in case of more parameters
        std::string portname = m_parameters->find("name").asString();

        m_rpcport = new yarp::os::RpcServer();
        m_rpcport->open(portname);
        
        m_mew_server.yarp().attachAsServer(*m_rpcport);
    }

}
