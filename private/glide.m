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

%% Glide
% Compute glide performance
% Adrien Crovato
%
% Speeds are computed assuming equilibrium along lift and drag directions,
% L = W * cos(gamma)
% D = W * sin(gamma)
function res = glide(prm, var, res)

% constants
w = prm.mtow * 9.81; % weight
rho = 1.225; % density
s = prm.S; % wing area
cd0 = prm.cd0; % zero-lift drag
k = 1 / (prm.eps * prm.ar * pi); % induced drag factor
csg = 1; % cosine of AOC (gamma << => csg = 1), assumed constant to have a linear equation in gamma

% variables
v = linspace(0, var.v_, 100) * 0.514; % equivalent airspeed
pdS = 0.5 * rho * v.^2 * prm.S; % equivalent dynamic pressure

% AOD and ROD
gamma = atan2(cd0 + k * csg^2 * w^2 ./ (pdS).^2, csg * w ./ pdS);
hDot = sin(gamma) .* v;

% Best glide speed and efficiency
vBest = sqrt(2 * w / rho / s * sqrt(k / cd0));
eff = 0.5 / sqrt(k * cd0);

% Minimum sink rate speed
vMin = sqrt(2 * w / (1.225 * s) * sqrt(k /(3 * cd0)));

% Assign
res.vg = vBest;
res.ve = vMin;
res.e = eff;

% Conversion
vs = res.vs / 0.514; 
v = v / 0.514; % kts
gamma = gamma * 180 / pi; % deg
hDot = hDot / 0.3048 * 60; % ft / min

% Display
disp(['Best glide speed = ', num2str(round(vBest / 0.514)), ' KEAS']);
disp(['Efficiency = ', num2str(round(eff, 1))]);
disp(['Minimum sink rate speed = ', num2str(round(vMin / 0.514)), ' KEAS']);

figure; hold on;
set(gca, 'XLim', [0, var.v_]);
set(gca, 'XTick', linspace(0, var.v_, 5));
set(gca,'Fontsize', 16,'Fontname', 'Times', 'LineWidth', 0.5);
yyaxis left
plot([vs vs], [0, 100], 'k-.', 'LineWidth', 1.5);
plot(v, gamma, 'LineWidth', 2.0);
xlabel('KEAS', 'Interpreter', 'Latex');
ylabel('$\gamma$', 'Interpreter', 'Latex');
yyaxis right
plot(v, hDot, 'LineWidth', 2.0);
ylabel('$\dot{h}$','Interpreter', 'Latex');
title('Angle and rate of descent');

end