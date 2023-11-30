classdef NAV_TYPE_MASK < int32
    enumeration
        POS_MASK         (3576)  % position control only
        VEL_MASK         (3527)  % velocity control only
        POSVEL_MASK      (3520)  % position + velocity control
    end
end