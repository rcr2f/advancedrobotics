
namespace stewart_platform
{

class InverseKinematics
{
   //servo/joint measurements
   float servoBasePos[]{308.5, 351.5, 68.5, 111.5, 188.5, 231.5 };
   float jointPlatformPos[]{286.10, 13.9, 46.1,  133.9, 166.1, 253.9};
   float servoArmAngle[]{-8*PI/3, PI/3, 0, -PI, -4*PI/3, -7*PI/3}; //beta

   //stewart platform size measurements
   float initialHeight = 250;
   float baseRadius = 140;
   float platformRadius = 32;
   float servoArmLength = 36;
   float legLength = 270;
   
   void calcLegLengths()
   {
   }

   void calcArmAngles()
   {

   }
};

}
