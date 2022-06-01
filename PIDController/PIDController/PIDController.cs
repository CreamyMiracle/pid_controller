namespace PID
{
    public class PIDController
    {
        private float proportionalGain;
        private float integralGain;
        private float derivativeGain;

        private float lastError;
        private float lastValue;

        private float integrationStored;
        private float integralSaturation;

        private float outMin;
        private float outMax;

        private DerivativeMeasurement derivativeMeasurement = DerivativeMeasurement.Error;
        private SystemType systemType = SystemType.Linear;

        private bool derivativeInitialized = false;

        public PIDController(float _proportionalGain, float _integralGain, float _derivativeGain, float _outMin, float _outMax, float _integralSaturation, DerivativeMeasurement _derivativeMeasurement = DerivativeMeasurement.Error, SystemType _systemType = SystemType.Linear)
        {
            proportionalGain = _proportionalGain;
            integralGain = _integralGain;
            derivativeGain = _derivativeGain;

            outMin = _outMin;
            outMax = _outMax;

            derivativeMeasurement = _derivativeMeasurement;

            systemType = _systemType;
            integralSaturation = _integralSaturation;
        }

        public float Update(float dTime, float currValue, float targetValue)
        {
            float currError = Error(currValue, targetValue);

            // Proportional to error
            float p_term = P(currError);

            // Continuos error elimination
            float i_term = I(currError, dTime);

            // Fights the rate of change of the error (or value)
            float dValue = ValueDiff(currValue, dTime);
            float dError = ErrorDiff(currError, dTime);
            float d_term = D(dError, dValue);

            float result = p_term + d_term + i_term;

            return Math.Clamp(result, outMin, outMax);
        }

        public void Reset()
        {
            derivativeInitialized = false;
            integrationStored = 0;
        }

        private float ValueDiff(float currValue, float dTime)
        {
            float dValue;
            if (systemType.Equals(SystemType.Angular))
            {
                dValue = AngularDifference(currValue, lastValue) / dTime;
            }
            else
            {
                dValue = LinearDifference(lastValue, currValue) / dTime;
            }
            lastValue = currValue;
            return dValue;
        }

        private float ErrorDiff(float currError, float dTime)
        {
            float dError;
            if (systemType.Equals(SystemType.Angular))
            {
                dError = AngularDifference(currError, lastError) / dTime;
            }
            else
            {
                dError = LinearDifference(lastError, currError) / dTime;
            }
            lastError = currError;
            return dError;
        }

        private float Error(float currValue, float targetValue)
        {
            if (systemType.Equals(SystemType.Angular))
            {
                return AngularDifference(currValue, targetValue);
            }
            else
            {
                return LinearDifference(currValue, targetValue);
            }
        }

        private float P(float error)
        {
            return proportionalGain * error;
        }

        private float I(float error, float dTime)
        {
            integrationStored = integrationStored + (error * dTime);
            return Math.Clamp(integralGain * integrationStored, -integralSaturation, integralSaturation);
        }

        private float D(float dError, float dValue)
        {
            if (!derivativeInitialized)
            {
                derivativeInitialized = true;
                return 0;
            }

            // To prevent derivative kicks
            if (derivativeMeasurement.Equals(DerivativeMeasurement.Value))
            {
                return derivativeGain * -dValue;
            }
            else
            {
                return derivativeGain * dError;
            }
        }

        private float LinearDifference(float currValue, float targetValue)
        {
            return targetValue - currValue;
        }
        
        private float AngularDifference(float currValue, float targetValue)
        {
            return (currValue - targetValue + 540) % 360 - 180;
        }
    }

    public enum DerivativeMeasurement
    {
        Value,
        Error
    }

    public enum SystemType
    {
        Linear,
        Angular
    }
}