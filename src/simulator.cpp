#include "simulator.h"
#include "FileReader.h"
#include "stuff.cpp"
#include "Eigen/Core"
#include <vector>

#define ROTATION_THRESHOLD
#define TRANSLATION_THRESHOLD

sf::RenderWindow _window;
sf::View _camera;
sf::Image _backgroundImage;
sf::Sprite _backgroundSprite;
sf::Image _robotImage;
sf::Sprite _robotSprite;
sf::Sprite _fakeRobotSprite;
sf::Image _landmarkImage;
sf::Sprite _landmarkSprite;
sf::Image _rangeImage;
sf::Sprite _rangeSprite;

RobotPose current_pose;
RobotPose last_captured_pose;
RobotPose fake_current_pose;
RobotPose fake_last_captured_pose;


int window_width = 1024;
int window_height = 700;


// configs
int FRAMES_PER_SECOND = 50;
float SKIP_TICKS = ((float)1)/((float)FRAMES_PER_SECOND);
int MAX_FRAMESKIP = 10;

double max_move_step = 4; // maximum "speed" value
double move_step_increment = 0.5;  // this tells how much does the robot move when it moves
double max_rotation_step = .05; // maximum rotation value
double rotation_step_increment = .01;  // this tells how much does the robot rotation change when it does
double move_trigger = 5; // distance to be walked in order to trigger a capture action
double rotation_trigger = 0.05; // rotation to be performed in order to trigger a capture action

double max_walk_error = 2; // maximum amount of 'walk error' in the robot guess
double max_rotation_error = 0.02; // maximum amount of 'rotation error' in the robot guess
double max_perception_error = 0.02; // maximum amount of error (radians) in the bearing sensor

double range = 50;  // this is the distance within which the robot is able to detect landmarks
double min_range = 20; // minimum value for 'range'
double max_range = 400; // maximum value for 'range'
bool time_to_capture; // boolean used to actually trigger the capture action

// other global variables declaration
double original_range_sprite_size_x = 0;
double original_range_sprite_size_y = 0;

double walk; // if positive, the robot moves
double rotation;  // rotation: if 0, the robot must not rotate, if positive it must turn left, if negative it must turn right

double fake_walk; // same as walk, but refers to the robot guess, not to the ground truth
double fake_rotation; // same as rotation, but refers to the robot guess, not to the ground truth

std::ofstream truth_out("truth.trj");
std::ofstream noised_out("noised.trj");

std::vector<Landmark> landmarks;  // this contains the landmarks
std::vector<Landmark *> detected_landmarks;  // this will be filled step by step with the currently in-range landmarks
std::vector<double> perceptions;  // this will be filled with the bearing at which the robot sees the landmark at the corresponding position in the detected_landmarks vector

double move_amount;
double rotation_amount;

// exit
void quit(){
  truth_out.close();
  noised_out.close();
  exit(0);
}

void updateRangeSpriteScale(){
  double scale_x = range/(original_range_sprite_size_x/2);
  double scale_y = range/(original_range_sprite_size_y/2);
  _rangeSprite.SetScale(scale_x, scale_y);
}

void init(){
  _backgroundImage.LoadFromFile("assets/wood-puzzle-floor.png");
  _backgroundSprite.SetImage(_backgroundImage);
  
  _robotImage.LoadFromFile("assets/roomba.png");
  _robotSprite.SetImage(_robotImage);
  _robotSprite.SetCenter(_robotSprite.GetSize().x/2, _robotSprite.GetSize().y/2);
  _fakeRobotSprite.SetImage(_robotImage);
  _fakeRobotSprite.SetCenter(_fakeRobotSprite.GetSize().x/2, _fakeRobotSprite.GetSize().y/2);
  _fakeRobotSprite.SetColor(sf::Color(100,100,255));

  _landmarkImage.LoadFromFile("assets/landmark.png");
  _landmarkSprite.SetImage(_landmarkImage);
  _landmarkSprite.SetCenter(_landmarkSprite.GetSize().x/2, _landmarkSprite.GetSize().y/2);

  _rangeImage.LoadFromFile("assets/range.png");
  _rangeSprite.SetImage(_rangeImage);
  original_range_sprite_size_x = _rangeSprite.GetSize().x;
  original_range_sprite_size_y = _rangeSprite.GetSize().y;
  updateRangeSpriteScale();
  _rangeSprite.SetCenter((original_range_sprite_size_x/2), (original_range_sprite_size_y/2));

  _window.Create(sf::VideoMode(window_width,window_height,32), "Bearing simulator");

  move_amount = 0;
  rotation_amount = 0;
  time_to_capture = false;
  
  walk = 0;
  rotation = 0;
}

void setDefaultLandmarkPositions(){
  landmarks.push_back(Landmark(-100,200));
  landmarks.push_back(Landmark(300,0));
  landmarks.push_back(Landmark(100,70));
}

void loadLandmarks(std::string landmarksFileName){
  FileReader fr(landmarksFileName);
  if(!fr.is_open()){
    std::cout << "landmarks file not located, putting landmarks in 'default' position" << std::endl;
    setDefaultLandmarkPositions();
    return;
  }
  
  std::vector<std::string> textline;
  fr.readLine(&textline);
  while(fr.good()){
    if(textline.size() > 1){
      double x = atof(textline[0].c_str());
      double y = atof(textline[1].c_str());
      landmarks.push_back(Landmark(x,y));
    }
    textline.clear();
    fr.readLine(&textline);
  }
}

void draw(){
  _window.Clear();
  
  {
    sf::Rect<float> frect = sf::Rect<float>(0,0,window_width,window_height);
    _camera.SetFromRect(frect);
  }
  
  _window.Draw(_backgroundSprite);
  
  
  // draw the range shade
  _rangeSprite.SetPosition(current_pose.x + window_width/2, window_height/2 - current_pose.y);
  _window.Draw(_rangeSprite);
  
  // draw the landmarks
  for(unsigned int i=0; i<landmarks.size(); i++){
    _landmarkSprite.SetPosition(landmarks[i].x + window_width/2, window_height/2 - landmarks[i].y);
    _window.Draw(_landmarkSprite);
  }

  // draw the robot
  _robotSprite.SetPosition(current_pose.x + window_width/2, window_height/2 - current_pose.y);
  _robotSprite.SetRotation(current_pose.theta * 180 / M_PI);
  _window.Draw(_robotSprite);
  
  // draw the "other" robot
  _fakeRobotSprite.SetPosition(fake_current_pose.x + window_width/2, window_height/2 - fake_current_pose.y);
  _fakeRobotSprite.SetRotation(fake_current_pose.theta * 180 / M_PI);
  _window.Draw(_fakeRobotSprite);
  
  // display everything
  _window.Display();
}

void mouseLeftPressed(float x, float y){
  landmarks.push_back(Landmark(x - (float)window_width/2 , ((float)window_height/2) - y));
}

int handleEvents(){
  sf::Event event;
  while(_window.GetEvent(event)){
    switch(event.Type)
      {
      case sf::Event::KeyPressed:
	switch(event.Key.Code){
	case sf::Key::Escape:
	  quit();
	  break;
	case sf::Key::Up:
	  walk += move_step_increment;
	  walk = std::min(walk, max_move_step);
	  break;
	case sf::Key::Down:
	  walk -= move_step_increment;
	  walk = std::max(walk, .0);
	  break;
	case sf::Key::Left:
	  rotation += rotation_step_increment;
	  rotation = std::min(rotation, max_rotation_step);
	  break;
	case sf::Key::Right:
	  rotation -= rotation_step_increment;
	  rotation = std::max(rotation, -max_rotation_step);
	  break;
	case sf::Key::Return:
	  rotation = 0;
	  walk = 0;
	  break;
	case sf::Key::Q:
	  range += 10;
	  range = std::min(range, max_range);
	  updateRangeSpriteScale();
	  break;
	case sf::Key::W:
	  range -= 10;
	  range = std::max(range, min_range);
	  updateRangeSpriteScale();
	  break;
	default:
	  break;
	}	// end of key switch
	break;
      case sf::Event::MouseButtonPressed:
	if(event.MouseButton.Button == sf::Mouse::Left){
	  mouseLeftPressed(event.MouseButton.X, event.MouseButton.Y);
	}
	break;
      default:
	break;
      }	// end of event type switch
  }	// end of while
  return 0;
}

void update(){
  // set fake walk/rotation values
  fake_walk = walk;
  if(fake_walk > 0){ // we assume that the robot understands that it is still, when it is
    fake_walk -= max_walk_error;
    fake_walk += ((double)(rand())/(double)(RAND_MAX)) * 2 * max_walk_error;
    if(fake_walk < 0){fake_walk = 0;}
  }
  fake_rotation = rotation;
  fake_rotation -= max_rotation_error;
  fake_rotation += ((double)(rand())/(double)(RAND_MAX)) * 2 * max_rotation_error;
  

  // update robot position and rotation
  if(walk > 0){
    // modify robot position
    current_pose.x = current_pose.x + cos(current_pose.theta) * walk;
    current_pose.y = current_pose.y + sin(current_pose.theta) * walk;
    current_pose.theta += rotation;
    current_pose.theta = normalizeAngle(current_pose.theta);
    
    // modify robot position guess
    fake_current_pose.x = fake_current_pose.x + cos(fake_current_pose.theta) * fake_walk;
    fake_current_pose.y = fake_current_pose.y + sin(fake_current_pose.theta) * fake_walk;
    fake_current_pose.theta += fake_rotation;
    fake_current_pose.theta = normalizeAngle(fake_current_pose.theta);

    move_amount += walk;
    rotation_amount += d_abs(rotation);
  }
  
  // check capture triggers
  if(move_amount > move_trigger || rotation_amount > rotation_trigger){
    // std::cout << std::endl;
    // std::cout << "moved: " << move_amount << "; rotated: " << rotation_amount << "; ";
    // if (move_amount > move_trigger){
    //   std::cout << "move trigger; ";
    // }
    // if (rotation_amount > rotation_trigger){
    //   std::cout << "rotation trigger; ";
    // }
    // std::cout << std::endl;
    move_amount = 0;
    rotation_amount = 0;
    time_to_capture = true;
  }
  
  // eventually capture
  if(time_to_capture){
    // reset time_to_capture
    time_to_capture = false;
    
    // detect in-range landmarks
    for(unsigned int i=0; i<landmarks.size(); i++){

      // compute the distance between the landmark and the robot
      double dx = landmarks[i].x - current_pose.x;
      double dy = landmarks[i].y - current_pose.y;
      double distance = sqrt(pow(dx,2) + pow(dy,2));
      
      // check if the landmark is within the range
      if(distance < range){
	detected_landmarks.push_back(&(landmarks[i]));
	perceptions.push_back(computeAngle(&landmarks[i], &current_pose));
      }
    }
    
    // compute transformation matrix between last capture pose and current pose (and do the same for the robot guess)
    Eigen::Matrix3d m1 = r2m(last_captured_pose);
    Eigen::Matrix3d m2 = r2m(current_pose);
    Eigen::Matrix3d fm1 = r2m(fake_last_captured_pose);
    Eigen::Matrix3d fm2 = r2m(fake_current_pose);
    RobotPose transform = m2r(m1.inverse()*m2);
    RobotPose fake_transform = m2r(fm1.inverse()*fm2);
    
    // write stuff on files
    truth_out << transform.x << " " << transform.y << " " << transform.theta;
    noised_out << fake_transform.x << " " << fake_transform.y << " " << fake_transform.theta;
    
    for(unsigned int i = 0; i<perceptions.size(); i++){
      truth_out << " " << perceptions[i];
      noised_out << " " << perceptions[i] - max_perception_error + 2 * max_perception_error * ((double)rand())/((double)RAND_MAX);
    }
    truth_out << std::endl;
    noised_out << std::endl;
    
    // clear the vectors
    detected_landmarks.clear();
    perceptions.clear();
    
    // update last_captured_pose
    last_captured_pose = current_pose;
    fake_last_captured_pose = fake_current_pose;
  }
}

static void tests(){
  return;
  
  sf::Clock clock;
  clock.Reset();
  for(int i=0; i<30; i++){
    std::cout << "elapsed: " << clock.GetElapsedTime() << std::endl;
  }
  
  RobotPose r1;
  r1.x = 1;
  r1.y = 0;
  r1.theta = 0;
  
  std::cout << "r1:" << std::endl << r1.x << "\t" << r1.y << "\t" << r1.theta << std::endl;
  std::cout << std::endl;
  
  Eigen::Matrix3d m1 = r2m(r1);
  std::cout << "m1:" << std::endl << m1 << std::endl;
  std::cout << std::endl;

  Eigen::Vector3d v1;
  v1[0] = 1;
  v1[1] = 0;
  v1[2] = 1;
  std::cout << "v1:" << std::endl << v1 << std::endl;
  std::cout << std::endl;

  std::cout << "m1*v1 = " << std::endl << m1*v1 << std::endl;
  std::cout << std::endl;
  
  RobotPose r2 = r1;
  r2.x += 1;
  r2.y += 1;
  r2.theta = M_PI_2;
  
  std::cout << "r2:" << std::endl << r2.x << "\t" << r2.y << "\t" << r2.theta << std::endl;
  std::cout << std::endl;
  
  Eigen::Matrix3d m2 = r2m(r2);
  std::cout << "m2:" << std::endl << m2 << std::endl;
  std::cout << std::endl;
  
  std::cout << "m1*m2*v1 = " << std::endl << m1*m2*v1 << std::endl;
  std::cout << std::endl;
  
  std:: cout << "m1.inverse() * m2 [relative_displacement]:" << std::endl << m1.inverse()*m2 << std::endl;
  std::cout << std::endl;
}

int main(int argc, char ** argv)
{
  
  // tests
  tests();
  
  // initialize stuff
  init();
  
  // create landmarks
  if(argc>1)
    loadLandmarks(std::string(argv[1]));
  else
    loadLandmarks("assets/landmarks.txt");

  // prepare variables for the main loop
  current_pose.x = 0;
  current_pose.y = 0;
  current_pose.theta = 0;
  last_captured_pose = current_pose;
  fake_current_pose = current_pose;
  fake_last_captured_pose = current_pose;
  double sleep_time = 0; // used for waiting next step
  
  sf::Clock clock;
  clock.Reset();
  float next_step_time = clock.GetElapsedTime();
  int loop = 0;
  while(true){
    loop = 0;
    
    while((clock.GetElapsedTime() > next_step_time) && (loop < MAX_FRAMESKIP)){
      next_step_time += SKIP_TICKS;
      handleEvents();
      update();
      loop++;
      sleep_time = next_step_time - clock.GetElapsedTime();
      if(sleep_time > 0){
	usleep(sleep_time*1000);
      }
    }
    draw();
  }
  
  return 0;
}
