from wpimath.filter import SlewRateLimiter

class Constants:
    class Limiters:
        translationRate = 2.5
        translationRateClose = 1.5
        
        omegaRate = 2.5
        omegaRateClose = 1.5

        srl_tX = SlewRateLimiter( translationRate )
        srl_tY = SlewRateLimiter( translationRate )
        srl_rO = SlewRateLimiter( omegaRate )
        
        srl_tX_close = SlewRateLimiter( translationRateClose )
        srl_tY_close = SlewRateLimiter( translationRateClose )
        srl_rO_close = SlewRateLimiter( omegaRateClose )