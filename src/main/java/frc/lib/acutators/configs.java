package frc.lib.acutators;

import com.revrobotics.SparkMaxPIDController;

public class configs {

    public void SLOT0(SparkMaxPIDController _controller, Gains _gains){
        _controller.setP(_gains.kp(),0);
        _controller.setI(_gains.ki(), 0);
        _controller.setD(_gains.kd(), 0);
        _controller.setFF(_gains.kff(), 0);
        _controller.setIZone(_gains.kIz(), 0);

        _controller.setSmartMotionMaxVelocity(_gains.SMMV(), 0);
        _controller.setSmartMotionMaxAccel(_gains.SMMA(), 0);

        _controller.setOutputRange(-1,0);
    }

    public void SLOT1(SparkMaxPIDController _controller, Gains _gains){
        _controller.setP(_gains.kp(),1);
        _controller.setI(_gains.ki(), 1);
        _controller.setD(_gains.kd(), 1);
        _controller.setFF(_gains.kff(), 1);
        _controller.setIZone(_gains.kIz(), 1);

        _controller.setSmartMotionMaxVelocity(_gains.SMMV(), 1);
        _controller.setSmartMotionMaxAccel(_gains.SMMA(), 1);

        _controller.setOutputRange(-1,0);
    }


}
