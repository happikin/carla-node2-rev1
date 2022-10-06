#include "carlabridge.h"

std::chrono::time_point<std::chrono::system_clock> simulationStartTime_global=std::chrono::system_clock::now();

void CarlaClient::pidController(){

	float Kp,Ki,Kd;
	float output=0;
	float currentAcceleration=0;
	float currentVelocity=0;
	float currentAccelerationSetpoint=0;
	carla::client::Vehicle::Control odom;
	odom.steer=0;
	
	Kp=10.0;
	//Ki=0.5;
	//Ki=0.3;
	////Ki=0.05;
	////Kd=0.001;
	//Ki=0.0007; // gives low but mostly steady acceleration
	Ki=0.0;
	Kd=0.0;
	//Kd=0.0005;

	std::fstream fio;
	float printingFloat,printingFloat1,printingFloat2;
	float dt=0.0;
	float velocitySetPoint=0.0;
	float a_dt=0.0;
	float yaw_des=0.0;
	float yaw_current=0.0;

	float error_vel=0.0;
	float currentVel_setpoint=0.0;
	float error_vel_dot=0.0;
	
	prev_time=std::chrono::system_clock::now();
	prev_error=0.0;
	while(stopListening_flag==0){

		carla::geom::Vector3D currentAccelerationVector=carlaCarActor_downcast->GetAcceleration();
		carla::geom::Vector3D currentVelocityVector=carlaCarActor_downcast->GetVelocity();
		currentAcceleration=std::sqrt((std::pow(currentAccelerationVector.x,2))+(std::pow(currentAccelerationVector.y,2)));
		yaw_des=atan2(currentAccelerationVector.y,currentAccelerationVector.x)*(180.0/3.14159265);
		yaw_current=carlaCarActor_downcast->GetTransform().rotation.yaw;
		if(std::fabs(yaw_des-yaw_current)>90.0) currentAcceleration=-currentAcceleration;

		currentVelocity=std::sqrt((std::pow(currentVelocityVector.x,2))+(std::pow(currentVelocityVector.y,2)));
		currentAccelerationSetpoint=accelerationSetpoint.load();
		accelerationSetpoint.store(0.0); //only use the fresh values
		error=currentAccelerationSetpoint-currentAcceleration;
		
		//if(prev_time==simulationStartTime){
		//	output=Kp*(error);
		//	dt=0.0;
	//	}
	//	else{
	//		sum_error+=((error)*dt);
	//		dt=(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-prev_time).count());
	//		dt=dt/1000.0;
	//		velocitySetPoint+=(currentAccelerationSetpoint*dt);
	//		output=/*Kp*(error)+Kd*((error-prev_error)/(dt))+*/Ki*(sum_error)+Kd*((error-prev_error)/dt)+Kp*(error);
	//	}
		dt=(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-prev_time).count())/1000.0;
		
		//if( (((error-prev_error)/dt)<=80) && (((error-prev_error)/dt)>=-80) ){
			//sum_error+=((error));
		 	//output=(Kp*error)+(Ki*sum_error)+(Kd*(error-prev_error)); //checking if the error aren't due to engine oscillations
		 //}
		 //currentVel_setpoint+=(currentAccelerationSetpoint*dt);
		 currentVel_setpoint=velocitySetpoint.load();
		 error_vel_dot=error_vel;
		 error_vel=currentVel_setpoint-currentVelocity;
		 //printf("\n currentVelocity=%f \n",currentVelocity);
		 error_vel_dot-=error_vel;
		 output=Kp*error_vel+Kd*error_vel_dot;
		//output=0.3;
		
		prev_time=std::chrono::system_clock::now();
		//error=velocitySetPoint-currentVelocity;
		//sum_error=((error));
		a_dt+=(currentAcceleration*dt);
		
		
		fio.open("accGraphValues.txt", std::ios::app);
		printingFloat=((int16_t)(1000.0*currentAcceleration));
		printingFloat1=((int16_t)(1000.0*currentVelocity));
		printingFloat2=((int16_t)(1000.0*a_dt));
		fio<<"currentAcceleration "<<printingFloat/1000.0<<" setpointAcceleration "<<currentAccelerationSetpoint<<" output "<<output<<" currentVelocity "<<printingFloat1/1000.0<<" timeStamp "
		                           <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-simulationStartTime).count()<<
		                           " error "<<error<<" error_derivative "<<Kd*(error-prev_error)<<" error_integrated "<<sum_error<<" setpointVelocity "<<currentVel_setpoint<<//velocitySetPoint<<
		                           " currentAcceleration_dt "<<(printingFloat2)/1000.0<<std::endl;
		fio.close();		
		

		//odom.manual_gear_shift=true;
		//odom.gear=1;
		if(currentAccelerationSetpoint>=0.0){
			if(output>1.0) output=1.0;
			else if(output<0.0) output=0.0;
			odom.throttle=output;
			odom.brake=0.0;
		}
		else if(currentAccelerationSetpoint<0.0){
			output=-output;

			if(output>1.0) output=1.0;
			else if(output<0.0) output=0.0;
			odom.throttle=0.0;
			odom.brake=output;			
		}
		carlaCarActor_downcast->ApplyControl(odom); //just for testing

		
		//printf("\n currentAcceleration=%f setpointAcceleration=%f currentVelocity=%f in PID loop\n",currentAcceleration,currentAccelerationSetpoint,currentVelocity);
		//printf("\n throttle=%f brake=%f \n",odom.throttle,odom.brake);

		prev_error=error;
		//sum_error+=(error*dt);

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void CarlaClient::ImgCbk(carla::SharedPtr<carla::sensor::SensorData> rawSensorDataPtr) {
	std::cout << "inside image callback\n";
	currentImageFrameVal++;
	carla::SharedPtr<carla::sensor::data::ImageTmpl<carla::sensor::data::Color>> ImagePtr 
		= boost::static_pointer_cast<carla::sensor::data::ImageTmpl<carla::sensor::data::Color>>(rawSensorDataPtr);
	
	long img_size
		= ImagePtr->GetHeight() *
		ImagePtr->GetWidth() * 3;

	uint8_t image_buffer[img_size];
	int index{0};
	for(auto it{ImagePtr->begin()}; it != ImagePtr->end(); it++) {
		// need to see how to write rgb values
		image_buffer[index + 0] = it->r;
		image_buffer[index + 1] = it->g;
		image_buffer[index + 2] = it->b;
		index += 3;
	}

	{
		std::lock_guard<std::mutex> lock_guard(m_image_mutex);
		m_image_data = cv::Mat(ImagePtr->GetHeight(),ImagePtr->GetWidth(),CV_8UC3,image_buffer);
	}
	// cv::imshow("tmp_image",m_image_data);
	// cv::waitKey(1);
}

void CarlaClient::radar_callback(carla::SharedPtr<carla::sensor::SensorData> rawSensorDataPtr){
	// std::cout << "reached radar_callback\n";
	carla::SharedPtr<carla::sensor::data::RadarMeasurement> RadarCloudPtr
		= boost::static_pointer_cast<carla::sensor::data::RadarMeasurement>(rawSensorDataPtr);

	{
		/**
		 * TODO: need to find out how to use all RadarDetections in a RadarMeasurement
		 * the below mentioned way is not good
		*/
		std::lock_guard<std::mutex> lock_guard(m_radar_mutex);
		carla::sensor::data::RadarDetection radar_detection = *RadarCloudPtr->begin();
		m_radar_data.altitude = radar_detection.altitude;
		m_radar_data.azimuth = radar_detection.azimuth;
		m_radar_data.depth = radar_detection.depth;
		m_radar_data.velocity = radar_detection.velocity;
		m_radar_data.ego_velocity
			= std::sqrt((std::pow(carlaCarActor_downcast->GetVelocity().x, 2))
            + (std::pow(carlaCarActor_downcast->GetVelocity().y, 2))
            + (std::pow(carlaCarActor_downcast->GetVelocity().z, 2)));

	}

}

void CarlaClient::updateCarCamera() {

	float angleRad;
	float temp_currentVelocity;
	float temp_currentVelocity_otherCar;
	while(stopListening_flag==0){
		spectator_transform=carlaCarActor->GetTransform();
		//angleRad=-carla::geom::Math::ToRadians(spectator_transform.rotation.yaw);
		angleRad=-carla::geom::Math::ToRadians(spectator_transform.rotation.yaw);
		spectator_location=spectator_transform.location;
		spectator_location.y=spectator_location.y+(7*sin(angleRad));
		spectator_location.x=spectator_location.x-(7*cos(angleRad));
		spectator_location.z=spectator_location.z+3;
		spectator_transform.location=spectator_location;
		//spectator->SetTransform(spectator_transform);
		
		//spectator_transform.rotation.yaw=(90.0)-spectator_transform.rotation.yaw;
		//spectator_transform.location.y=spectator_transform.location.y+15;
		//spectator_transform.location.x=spectator_transform.location.x-10;
		
		//spectator_transform.location.y=spectator_location.y-(0.70*7*sin(angleRad));
		//spectator_transform.location.x=spectator_location.x+(0.70*7*cos(angleRad));
		//spectator_transform.location.z=spectator_location.z-0.5;
		temp_currentVelocity=std::sqrt(std::pow(carlaCarActor->GetVelocity().x,2)+std::pow(carlaCarActor->GetVelocity().y,2));
		temp_currentVelocity_otherCar=std::sqrt(std::pow(otherCarActor->GetVelocity().x,2)+std::pow(otherCarActor->GetVelocity().y,2));

		spectator->SetTransform(spectator_transform);
		spectator_location.x+=(5*cos(angleRad));
		dh.DrawString(
		carla::geom::Location(carlaCarActor->GetLocation().x-2.0,carlaCarActor->GetLocation().y-1.0,carlaCarActor->GetLocation().z+1.0), std::to_string(temp_currentVelocity),false,carla::sensor::data::Color(255u,0u,0u,255u),0.001,true);
		dh.DrawString(
		carla::geom::Location(otherCarActor->GetLocation().x-2.0,otherCarActor->GetLocation().y-1.0,otherCarActor->GetLocation().z+1.0), std::to_string(temp_currentVelocity_otherCar),false,carla::sensor::data::Color(255u,0u,0u,255u),0.001,true);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

void CarlaClient::generateWorld() {

	controllerThreadFlag=0;

	carla::client::World carlaWorld = carlaClient.GetWorld();
	currentMap = carlaWorld.GetMap();

	printf("\n checking map... \n");

	//if(currentMap->GetName()!="Carla/Maps/OpenDriveMap"){
	std::cout<<"currentMap->GetName()"<<currentMap->GetName()<<std::endl;
	//if(currentMap->GetName()!="map_package/Maps/StraightHighway_cruise/StraightHighway_cruise"){
		printf("\n Loading new map \n");
		//carlaClient.LoadWorld("OpenDriveMap");
		carlaClient.LoadWorld("/Game/map_package/Maps/StraightHighway_cruise/StraightHighway_cruise");
		// carlaClient.LoadWorld(carlaClient.GetAvailableMaps()[0]);

		std::this_thread::sleep_for(std::chrono::seconds(5));
	//}
	//else printf("\n map compatible \n");

	//carla::SharedPtr<carla::client::ActorList> carlaAllActors=carlaWorld.GetActors();
	carla::SharedPtr<carla::client::ActorList> carlaCarActors=carlaWorld.GetActors()->Filter("vehicle.*");
	carla::SharedPtr<carla::client::ActorList> carlaCamActors = carlaWorld.GetActors()->Filter("sensor.camera.rgb");
	carla::SharedPtr<carla::client::ActorList> carlaRadarActors = carlaWorld.GetActors()->Filter("sensor.other.radar");//("sensor.camera.rgb");

	//carla::SharedPtr<carla::client::Vehicle> otherCarActor; made this object a member variable.

	//carlaCarActors = ()->Filter("vehicle.*");
	spectator=carlaWorld.GetSpectator();	
	printf("\n checking car... \n");
	
	if(carlaCarActors->size()==1){
		printf("\n 1 car available.. \n");
		
		carlaCarActor = carlaCarActors->at(0);
		carlaCarActor_downcast=boost::static_pointer_cast<carla::client::Vehicle>(carlaCarActor);
		//otherCarActor= boost::static_pointer_cast<carla::client::Vehicle>(carlaCarActors->at(1));
		carlaCarActor_downcast->Destroy();
		//otherCarActor->Destroy();
	}
	else if(carlaCarActors->size()==2){
		printf("\n 2 cars available.. \n");

		carlaCarActor = carlaCarActors->at(0);
		carlaCarActor_downcast=boost::static_pointer_cast<carla::client::Vehicle>(carlaCarActor);
		otherCarActor= boost::static_pointer_cast<carla::client::Vehicle>(carlaCarActors->at(1));
		carlaCarActor_downcast->Destroy();
		otherCarActor->Destroy();	
	}
	//else{
	printf("\n spawning car \n");
	carla::SharedPtr<carla::client::BlueprintLibrary> carBlueprintPtr=carlaWorld.GetBlueprintLibrary()->Filter("vehicle.*");
	//carla::client::BlueprintLibrary carBlueprint = *carBlueprintList;
	//carla::client::BlueprintLibrary carBlueprint = carlaWorld.GetBlueprintLibrary()->Filter("vehicle.*")[0];
	carla::client::BlueprintLibrary::const_reference carActorBlueprint=carBlueprintPtr->at(0);

	carla::geom::Transform spectator_spawn_point=spectator->GetTransform();
	spectator_spawn_point.location.z+=2;
	//spectator_spawn_point.rotation.yaw+=90.0;
	spectator_spawn_point.rotation.yaw+=89.6;
	spectator->SetTransform(spectator_spawn_point);

	std::this_thread::sleep_for(std::chrono::seconds(1));

	carla::geom::Transform spawn_point=spectator->GetTransform();
	carla::geom::Transform spawn_point1=spectator->GetTransform();
	//spawn_point1.location.x+=10;
	
	spawn_point1.location.y+=25;
	//spawn_point1.location.y+=10; //just for testing
		
	spawn_point.location.z=2;
	spawn_point.location.x+=10;
	spawn_point1.location.z=2;
	spawn_point1.location.x+=10;

	//carla::geom::Transform spawn_point=carlaWorld.GetMap()->GetRecommendedSpawnPoints()[0];
	//carlaCarActor = carlaWorld.SpawnActor(carActorBlueprint,carTransform);
	
	carlaCarActor = carlaWorld.SpawnActor(carActorBlueprint,spawn_point);
	carlaCarActor_downcast
		= boost::static_pointer_cast<carla::client::Vehicle>(carlaCarActor);
	//otherCarActor = carlaWorld.SpawnActor(carBlueprintPtr->at(1),spawn_point);
	
	otherCarActor=boost::static_pointer_cast<carla::client::Vehicle>(carlaWorld.SpawnActor(carBlueprintPtr->at(2),spawn_point1));
	//}

	if(carlaRadarActors->size()!=0){
		printf("\n destroying old radar... \n");
		carlaRadarActors->at(0)->Destroy();
	}
	printf("\n spawning radar \n");
	carla::SharedPtr<carla::client::BlueprintLibrary> radarBlueprintPtr=carlaWorld.GetBlueprintLibrary()->Filter("sensor.other.radar");//("sensor.camera.rgb");
	auto radarBlueprint=radarBlueprintPtr->at(0);
	radarBlueprint.SetAttribute("range","100");
	radarBlueprint.SetAttribute("vertical_fov","15");
	radarBlueprint.SetAttribute("horizontal_fov","20"); //changing from 20 to 10 to get better performance at long range
	radarBlueprint.SetAttribute("points_per_second","1500");
	//radarBlueprint.SetAttribute("sensor_tick","0.05");
	
	carla::geom::Transform radarTransform(carla::geom::Location(carla::geom::Vector3D(2,0,1)),carla::geom::Rotation(0,0,0)); //-5 gives good concentration,-15
	//carla::geom::Transform radarTransform(carla::geom::Location(carla::geom::Vector3D(2,0,1)),carla::geom::Rotation(5,0,0));

	//carla::SharedPtr<carla::client::Actor> temporaryActor_radar=carlaWorld.SpawnActor(radarBlueprintPtr->at(0),radarTransform,carlaCarActor_downcast.get());
	carla::SharedPtr<carla::client::Actor> temporaryActor_radar=carlaWorld.SpawnActor(radarBlueprint,radarTransform,carlaCarActor_downcast.get());
	//carlaRadarActor = boost::static_pointer_cast<carla::client::ClientSideSensor>(temporaryActor_radar);
	carlaRadarActor = boost::static_pointer_cast<carla::client::ServerSideSensor>(temporaryActor_radar);
	
	if(carlaCamActors->size()!=0){
		printf("\n destroying old camera... \n");
		carlaCamActors->at(0)->Destroy();
	}

	printf("\n spawning camera \n");
	carla::SharedPtr<carla::client::BlueprintLibrary> cameraBlueprintPtr=carlaWorld.GetBlueprintLibrary()->Filter("sensor.camera.rgb");
	carla::geom::Transform camTransform(carla::geom::Location(carla::geom::Vector3D(2,0,1)),carla::geom::Rotation(0,0,0));

/******************************************** changing image resolution **************************************************/
	auto blueprint=cameraBlueprintPtr->at(0);//.begin();
	blueprint.SetAttribute("image_size_x","300");
	blueprint.SetAttribute("image_size_y","200");
	blueprint.SetAttribute("sensor_tick","0.0005");
	
	//carla::SharedPtr<carla::client::Actor> temporaryActor_camera=carlaWorld.SpawnActor(cameraBlueprintPtr->at(0),camTransform,carlaCarActor_downcast.get());
	carla::SharedPtr<carla::client::Actor> temporaryActor_camera=carlaWorld.SpawnActor(blueprint,camTransform,carlaCarActor_downcast.get());
	//carlaCamActor = boost::static_pointer_cast<carla::client::ClientSideSensor>(temporaryActor_camera);
	carlaCamActor = boost::static_pointer_cast<carla::client::ServerSideSensor>(temporaryActor_camera);

	std::thread spectatorThread(&CarlaClient::updateCarCamera,this);
	spectatorThread.detach();
	//spectator->SetTransform((carlaRadarActor->GetTransform()));
	std::cout << "before init sensor()\n";
	// initializeCarlaSensorData();
	std::cout << "after init sensor()\n";

	//std::function<void (carla::SharedPtr<carla::sensor::data::ImageTmpl<carla::sensor::data::Color>>)> localFunctorImg=std::bind(&CarlaClient::imgCbk,this,std::placeholders::_1);
	carla::client::Sensor::CallbackFunctionType functorRadar = std::bind(&CarlaClient::radar_callback,this,std::placeholders::_1);
	carlaRadarActor->Listen(functorRadar);
	std::cout << "after functor radar\n";

	carla::client::Sensor::CallbackFunctionType functorImg = std::bind(&CarlaClient::ImgCbk,this,std::placeholders::_1);
	carlaCamActor->Listen(functorImg);

	std::cout << "after functor image\n";
	// std::thread ddsTransmitThread(&CarlaClient::ddsTransmitFn,this);
	// ddsTransmitThread.detach();

	std::thread dds_node_thread(&CarlaClient::dds_node_proc,this);
	dds_node_thread.detach();

	std::thread vehicle_control_thread(&CarlaClient::vehicle_control_proc,this);
	vehicle_control_thread.detach();

	std::cout << "after dds thread\n";
	std::this_thread::sleep_for(std::chrono::seconds(1));
	
	std::cout << "after sleep\n";
	
	carla::client::Vehicle::Control odometryData;
	odometryData.throttle=0.3;
	std::cout << "before apply control\n";

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// other car apply control
	otherCarActor->ApplyControl(odometryData);

	std::cout << "after apply control\n";
	std::this_thread::sleep_for(std::chrono::seconds(1));
	
	printf("\n functorRadar ready \n");
}

CarlaClient::~CarlaClient() {}

CarlaClient::CarlaClient(
	const char* clientIP,
	int clientPort,
	int argc,
	char **argv
) : IP(clientIP), port(clientPort),
	carlaClient(IP,port),
	dh(carlaClient.GetWorld().MakeDebugHelper()) {

	this->argc = argc;
	this->argv = argv;
	currentImageFrameVal=0;
	latestFrame.store(0.0);
	stopListening_flag=0;
	error=0;
	prev_error=0;
	sum_error=0;
	std::fstream fio;
	fio.open("accGraphValues.txt", std::ios::trunc | std::ios::out);
	fio.close();
	simulationStartTime=std::chrono::system_clock::now();
	prev_time=simulationStartTime;
	generateWorld();
}

void CarlaClient::StopListening() {
	carlaCamActor->Stop();
	carlaRadarActor->Stop();
	stopListening_flag=1;
}

void CarlaClient::signal_handler(int _signal) {
	this->close_flag.store(1);
	std::cout << "\n ran signal_handler()\n";
}

void signal_callback_handler(int signum) {
   std::cout << "Caught signal " << signum << std::endl;
   // Terminate program
   terminateFlag=1;
   //exit(signum);
}

void CarlaClient::dds_node_proc() {
	std::cout << "running dds_node_thread\n";
	dds_node this_node(this->argc,this->argv);

	this_node.create_topic<
		CarlaData::RadarSensorTypeSupport_ptr,
		CarlaData::RadarSensorTypeSupportImpl
	> (topic_names[0].c_str());
	this_node.create_topic<
		CarlaData::VehicleOdometryTypeSupport_ptr,
		CarlaData::VehicleOdometryTypeSupportImpl
	> (topic_names[1].c_str());
	this_node.create_topic<
		CarlaData::ImageSensorTypeSupport_ptr,
		CarlaData::ImageSensorTypeSupportImpl
	> (topic_names[2].c_str());

	dds_publisher publisher(this_node);
	dds_subscriber subscriber(this_node);

	VehicleOdometryListener *odometry_listener = new VehicleOdometryListener();

	publisher.create_writer<CarlaData::RadarSensorDataWriter>(
		this_node, topic_names[0]);
	publisher.create_writer<CarlaData::ImageSensorDataWriter>(
		this_node, topic_names[2]);
	subscriber.create_reader<CarlaData::VehicleOdometryDataReader>(
		this_node, topic_names[1],(odometry_listener));

	std::thread([&](){
		publisher.wait_for_subscriber(topic_names[0]);
		while(true) {
			{
				std::lock_guard<std::mutex> lock_guard(m_radar_mutex);
				publisher.write<
					CarlaData::RadarSensor,
					CarlaData::RadarSensorDataWriter>
				(m_radar_data, topic_names[0]);
			}
			std::this_thread::sleep_for(
				std::chrono::microseconds(3000)
			);
		}
		publisher.wait_for_acknowledgments(topic_names[0]);
	}).detach();
	
	std::thread([&](){
		// cv::Mat read_image = cv::imread("/home/fev/Pictures/vibe1.png");
		std::cout << "before wait\n";
		publisher.wait_for_subscriber(topic_names[2]);
		std::cout << "after wait\n";
		while(close_flag.load() == 0) {
			std::cout << "in writer(0)\n";
			{
				std::lock_guard<std::mutex> lock_guard(m_image_mutex);
				if(m_image_data.size().height > 0 && m_image_data.size().width > 0) {
					CarlaData::ImageSensor image;
					long image_size = m_image_data.rows * m_image_data.cols * m_image_data.elemSize();

					image.height = m_image_data.rows;
					image.width = m_image_data.cols;
					image.pixel_size = m_image_data.elemSize();
					image.image_type = m_image_data.type();
					image.raw_data.replace(image_size, image_size, m_image_data.data, false);
					// cv::waitKey(1000);
					std::cout << "in writer()\n";
					publisher.write<CarlaData::ImageSensor,CarlaData::ImageSensorDataWriter>(
						image, topic_names[2]);
				}
			}
			std::this_thread::sleep_for(
				std::chrono::microseconds(250)
			);

			// exit(1);
		}
		publisher.wait_for_acknowledgments(topic_names[2]);
	}).detach();

	std::thread([&](){
		while(true) {
			m_odometry_data = odometry_listener->get_odometry();
			std::this_thread::sleep_for(
				std::chrono::microseconds(500)
			);
		}
	}).detach();

	std::thread sub_thread([&](){
		subscriber.wait_for_publisher(topic_names[1]);
		std::cout << "got publisher\n";
	});
	while(terminateFlag == 0);
	sub_thread.detach();
}

void CarlaClient::vehicle_control_proc() {
	while(true) {
		carla::rpc::VehicleControl control;
		control.brake = m_odometry_data.brake;
		control.throttle = m_odometry_data.throttle;
		control.steer = m_odometry_data.steering;
		carlaCarActor_downcast->ApplyControl(control);
		std::cout << "throttle: " << control.throttle << "\n";
		std::this_thread::sleep_for(
			std::chrono::microseconds(500)
		);
	}
}


int main(int argc, char** argv){

	// char *carlaBridge_argv[3]={"hvac_simulator.cpp", "-DCPSConfigFile", "/home/sdv/yashesvi/carla-ddsapi-experiment-1/rtps.ini"};
	// ddsNode_CarlaBridge obj(domainId,carlaBridge_argc,carlaBridge_argv);
	CarlaClient cc("172.22.236.183", 2000,argc,argv);
	signal(SIGINT, signal_callback_handler);
	while(terminateFlag==0){
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	// obj.StopListening_CarlaClient();
	// LoggerClass::writeFPS_2_document();
	return(0);
}
