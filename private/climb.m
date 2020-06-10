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

function res = climb(prm, var, res)
%% Climb
% Compute the climb performance as a function of the velcity and the altitude
% Adrien Crovato
%
% Climb is computed assuming equilibrium along lift and drag directions,
% L - cos(gamma) * W = 0
% T - D - W * sin(gamma) = 0
% AOC: sin(gamma) = 1/W * (T - D)
% ROC: hDot = sin(gamma) * V

% constants
w = prm.mtow * 9.81; % weight
s = prm.S; % wing area
cd0 = prm.cd0; % zero-lift drag
k = 1 / (prm.eps * prm.ar * pi); % induced drag factor
csg = 1; % square cosine of AOC (gamma << => csg = 1), assumed constant to have a linear equation in gamma

% variables
v_ = linspace(0, var.v_, 100)' * 0.514;
h_ = linspace(0, var.h_, 25) * 0.3048;

v = repmat(v_, 1, length(h_)); % equivalent airspeed
h = repmat(h_, length(v_), 1); % altitude
pdS = 0.5 * 1.225 * v.^2 * prm.S; % equivalent dynamic pressure
t = zeros(length(v_), length(h_)); % thurst
for i = 1:length(v_)
    for j = 1:length(h_)
        t(i,j) = thrust(v_(i), h_(j), prm);
    end
end

% AOC and ROC
sg = 1 / w * (t - pdS .* (cd0 + k * (w^2*csg^2) ./ (pdS.^2)));
sg(sg < 0 | isnan(sg)) = 0; % negative values correspond to a descent
gamma = asin(sg);
hDot = sg .* v;

% Airspeed for BAOC/BROC
% Analytic formula are not accurate...
% if prm.jet
%     sg2 = (prm.throttle / w - sqrt(4 * k * cd0))^2;
%     vX = sqrt(2 * w / (1.225 * s) * sqrt(k/cd0) * sqrt(1 - sg2));
%     vY = sqrt(prm.throttle / (1.225 * s * cd0) * (1 + sqrt(1 + 12 * k * cd0 / (prm.throttle / w)^2)));
% else
%     vX = fsolve(@(x) baocp(x, s, cd0, k, w, prm.throttle), var.v_, optimset('Display', 'off'));
%     vY = sqrt(2 * w / (1.225 * s) * sqrt(k /(3 * cd0)));
% end
[~, index] = max(gamma(:,1));
vX = v_(index);
[~, index] = max(hDot(:,1));
vY = v_(index);

% Assign
res.vx = vX; % best AOC speed
res.vy = vY; % best ROC speed

% Conversion
vs = res.vs / 0.514; % kts
v = v / 0.514; % kts
h = h / 0.3048; % ft
gamma = gamma * 180 / pi; % deg
hDot = hDot / 0.3048 * 60; % ft / min

% Display
disp(['Best AOC speed = ', num2str(round(vX / 0.514)), ' KEAS']);
disp(['Best ROC speed = ', num2str(round(vY / 0.514)), ' KEAS']);

figure; hold on;
set(gca, 'XLim', [0, var.v_]);
set(gca, 'YLim', [0, var.h_]);
set(gca, 'XTick', linspace(0, var.v_, 5));
set(gca, 'YTick', linspace(0, var.h_, 5));
set(gca,'Fontsize', 16,'Fontname', 'Times', 'LineWidth', 0.5);
plot([vs vs], [0, var.h_], 'k-.', 'LineWidth', 1.5);
[C, hdl] = contour(v, h, gamma);
clabel(C, hdl);
xlabel('KEAS', 'Interpreter', 'Latex');
ylabel('$h$', 'Interpreter', 'Latex');
title('Angle of climb');

figure; hold on;
set(gca, 'XLim', [0, var.v_]);
set(gca, 'YLim', [0, var.h_]);
set(gca, 'XTick', linspace(0, var.v_, 5));
set(gca, 'YTick', linspace(0, var.h_, 5));
set(gca,'Fontsize', 16, 'Fontname', 'Times', 'LineWidth', 0.5);
plot([vs vs], [0, var.h_], 'k-.', 'LineWidth', 1.5);
[C, hdl] = contour(v, h, hDot);
clabel(C, hdl);
xlabel('KEAS', 'Interpreter', 'Latex');
ylabel('$h$','Interpreter', 'Latex');
title('Rate of climb');
end

% BAOC airspeed for piston engine
function f = baocp(v, s, cd0, k, w, th)
p = th * 745.7 * 0.85; % max power for propeller engine
f = v^4 + p * v / (1.225 * s * cd0) - 4 * (w/s)^2 * k / (1.225^2 * cd0);
end