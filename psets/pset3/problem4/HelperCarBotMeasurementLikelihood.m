function likelihood = HelperCarBotMeasurementLikelihood(pf, predictParticles, measurement, sd) %#ok<INUSL>
%   HelperCarBotMeasurementLikelihood  Importance weight W(x;y) for each particle.
%
%   The GPS measures the pose directly:  y_t = h(X_t) + v_t,  v_t ~ N(0, sd^2 I).
%   The weight is the measurement-noise pdf evaluated at the residual:
%       W = f_V( y_t - h(x) )  proportional to  exp( -||y - h(x)||^2 / (2 sd^2) )
%

    if nargin < 4 || isempty(sd)
        sd = 0.5;  
    end

    err        = bsxfun(@minus, predictParticles(:,1:3), measurement);
    err(:,3)   = mod(err(:,3) + pi, 2*pi) - pi;

    likelihood = exp( -0.5 * sum(err.^2, 2) / sd^2 );

end
