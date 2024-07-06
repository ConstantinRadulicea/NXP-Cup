#ifndef __PID_H__
#define __PID_H__

class PID
{
public:
	PID(double max_output, double min_output, double Kp, double Ki, double Kd) :
        _max_output(max_output),
        _min_output(min_output),
        _Kp(Kp),
        _Kd(Kd),
        _Ki(Ki),
        _pre_error(0.0),
        _integral(0.0)
    {

	}
    double calculate(double setpoint, double pv, double timePassedFromLastSample)
    {
        // Calculate error
        double error = setpoint - pv;

        // Proportional term
        double Pout = _Kp * error;

        // Integral term
        _integral += error * timePassedFromLastSample;
        double Iout = _Ki * _integral;

        // Derivative term
        double derivative = (error - _pre_error) / timePassedFromLastSample;
        double Dout = _Kd * derivative;

        // Calculate total output
        double output = Pout + Iout + Dout;

        // Restrict to max/min
        if (output > _max_output)
            output = _max_output;
        else if (output < _min_output)
            output = _min_output;

        // Save error to previous error
        _pre_error = error;

        return output;
    }
	~PID() {

	}

    void setMaxOutput(double val) {
        this->_max_output = val;
    }
    void setMinOutput(double val) {
        this->_min_output = val;
    }

    void setKp(double val) {
        this->_Kp = val;
    }

    void setKi(double val) {
        this->_Ki = val;
    }

    void setKd(double val) {
        this->_Kd = val;
    }


    double getMaxOutput(double val) {
        return this->_max_output;
    }
    double getMinOutput(double val) {
        return this->_min_output;
    }

    double getKp(double val) {
        return this->_Kp;
    }

    double getKi(double val) {
        return this->_Ki;
    }

    double getKd(double val) {
        return this->_Kd;
    }

    void reset() {
        this->_pre_error = 0.0;
        this->_integral = 0.0;
    }

private:
    double _max_output;
    double _min_output;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral;
};

#endif // !__PID_H__
