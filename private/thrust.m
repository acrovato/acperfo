% Copyright 2020 Adrien Crovato
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

function t = thrust(v, h, prm)
%% Thrust
% Compute the thurst as a function of the velocity and the altitude
% Adrien Crovato
%
% Todo: replace atmoisa by own function or open-source project
% Todo: use v and h as vector/matrix to reduce computational cost

if prm.jet
    [T, ~, P, ~] = atmosisa(h);
    thrust = prm.throttle * P / 101325 .* sqrt(288.15 ./ T); % N, (NASA)
    if v ~= 0; t = thrust; else; t = 0.5 * thrust; end % max thrust for jet
else
    [~, ~, ~, rho] = atmosisa(h);
    power = prm.throttle * (rho / 1.225 - (1 - rho / 1.225) / 7.55) * 745.7; % Watt (Raymer)
    if v ~= 0
        t = fsolve(@(x) propt(x, v, power, prm.dblade), power/v, optimset('Display', 'off')); % max thrust for propeller
    else
        t = power * 0.45 * 0.05; % 5% of static thrust (eta assumed to 0.45)
    end
end
end

% Thrust for propeller engine
function f = propt(t, v, p, d)
    a = pi * d^2 / 4;
    f = p - 0.5 * t * v * (sqrt(t / (0.5 * 1.225 * v^2 * a) + 1) + 1);
end