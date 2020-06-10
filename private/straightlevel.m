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

function res = straightlevel(prm, var, res)
%% Straight and level flight
% Compute stall and maximum speed
% Adrien Crovato
%
% Speeds are computed assuming equilibrium along lift and drag directions,
% L = W
% T = D

% constants
w = prm.mtow * 9.81; % weight
rho = 1.225; % density
s = prm.S; % wing area
clMax = prm.clmax; % max lift coefficient
cd0 = prm.cd0; % zero-lift drag
k = 1 / (prm.eps * prm.ar * pi); % induced drag factor

% stall stall speed
vStall = sqrt(2 * w / (rho * s * clMax));

% maximum speed
vMax = fsolve(@(x) equil(x, rho, k, w, prm), var.v_, optimset('Display', 'off')); % only at seal level

% Assign results
res.vs = vStall;
res.vc = vMax;

% Display
disp(['Stall speed = ', num2str(round(vStall / 0.514)), ' KEAS']);
disp(['Maximum speed = ', num2str(round(vMax / 0.514)), ' KEAS']);

end

function f = equil(v, rho, k, w, prm)
t = thrust(v, 0, prm);
f = (0.5 * rho * v^2 * prm.S)^2 * prm.cd0 - (0.5 * rho * v^2 * prm.S) * t - k * w^2;
end