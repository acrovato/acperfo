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

function res = turn(prm, var, res)
%% Turn
% Compute the turn performance as a function of the velcity and the altitude
% Adrien Crovato
%
% Climb is computed assuming equilibrium along lift, drag and turn directions,
% L * cos(phi) - W = 0
% T - D = Ps * W / v
% L * sin(phi) = m * v^2 / R
% rate: omega = g / v * sqrt(n^2 - 1)

% constants
w = prm.mtow * 9.81; % weight
s = prm.S; % wing area
clmax = prm.clmax; % maximum lift coefficient
k = 1 / (prm.eps * prm.ar * pi); % induced drag factor
nmax = prm.nmax; % maximum load factor

% variables
v = linspace(0, var.v_, 100)' * 0.514;
h = linspace(0, var.h_, 5) * 0.3048; % altitude
ps = linspace(-var.ps_, var.ps_, 3) * 0.3048; % excess power

% sustained turn rate as function of altitude
omega_h = zeros(length(v), length(h));
for i = 1 : length(h)
    omega_h(:,i) = turnrate(v, h(i), 0, w, k, prm);
end
% sea-level turn rate as function of excess power
omega_ps = zeros(length(v), length(ps));
for i = 1 : length(ps)
    omega_ps(:,i) = turnrate(v, 0, ps(i), w, k, prm);
end
% limits
vA = sqrt(2 * nmax * w / (1.225 * s * clmax));
v_l = [res.vs:vA]; % stall to n_max
v_n = [vA:res.vc]; % n_max to v_max
omega_l = 9.81 ./ v_l .* sqrt((0.5 * 1.225 * v_l.^2 * s).^2 * clmax ^2 / w^2 - 1); % aerodynamic
omega_n = 9.81 ./ v_n * sqrt(nmax^2 - 1); % structural

% Assign
res.va = vA;

% Conversion
v = v / 0.514; % kts
h = h / 0.3048; % ft
ps = ps / 0.3048; % ft/s
v_l = v_l / 0.514; % kts
v_n = v_n / 0.514; % kts
omega_n = omega_n * 180 / pi; % deg/s
omega_l = omega_l * 180 / pi; % deg/s
omega_h = omega_h * 180 / pi; % deg/s
omega_ps = omega_ps * 180 / pi; % deg/s

% Display
disp(['Maneuver speed = ', num2str(round(vA / 0.514)), ' KEAS']);

figure; hold on;
set(gca,'Fontsize', 16, 'Fontname', 'Times', 'LineWidth', 0.5);
set(gca, 'XLim', [0, var.v_]);
set(gca, 'XTick', linspace(0, var.v_, 5));
plot(v_l, omega_l, 'k-.', 'LineWidth', 1.5);
plot(v_n, omega_n, 'k-.', 'LineWidth', 1.5);
plot([v_n(end) v_n(end)], [omega_n(end), 0], 'k-.', 'LineWidth', 1.5);
for i = 1 : length(h)
    plot(v, omega_h(:,i), 'LineStyle', '-', 'LineWidth', 2.0);
end
legend('stall', ['n = ' num2str(nmax)], 'vmax (SL)', ['h = ' num2str(h(1)) ' ft'], ['h = ' num2str(h(2)) ' ft'], ['h = ' num2str(h(3)) ' ft'], ['h = ' num2str(h(4)) ' ft'], ['h = ' num2str(h(5)) ' ft']);
xlabel('KEAS', 'Interpreter', 'Latex');
ylabel('$\omega$','Interpreter', 'Latex');
title('Sustained turn rate');

figure; hold on;
set(gca,'Fontsize', 16, 'Fontname', 'Times', 'LineWidth', 0.5);
set(gca, 'XLim', [0, var.v_]);
set(gca, 'XTick', linspace(0, var.v_, 5));
plot(v_l, omega_l, 'k-.', 'LineWidth', 1.5);
plot(v_n, omega_n, 'k-.', 'LineWidth', 1.5);
plot([v_n(end) v_n(end)], [omega_n(end), 0], 'k-.', 'LineWidth', 1.5);
set(gca, 'ColorOrderIndex', 2);
plot(v, omega_ps(:,1), 'LineStyle', '--', 'LineWidth', 2.0);
set(gca, 'ColorOrderIndex', 1);
plot(v, omega_ps(:,2), 'LineStyle', '-', 'LineWidth', 2.0);
set(gca, 'ColorOrderIndex', 2);
plot(v, omega_ps(:,3), 'LineStyle', '-', 'LineWidth', 2.0);
legend('stall', ['n = ' num2str(nmax)], 'vmax (SL)', ['PS = ' num2str(ps(1))], ['PS = ' num2str(ps(2))], ['PS = ' num2str(ps(3))]);
xlabel('KEAS', 'Interpreter', 'Latex');
ylabel('$\omega$','Interpreter', 'Latex');
title('Instantaneous turn rate at sea level');

end

% turn rate
function omega = turnrate(v, h, ps, w, k, prm)
pdS = 0.5 * 1.225 * v.^2 * prm.S; % dynamic pressure X surface
% thurst
t = zeros(length(v),1);
for i = 1:length(v)
    t(i) = thrust(v(i), h, prm);
end

n = pdS.^2 / w^2 / k .* ((t - ps * w ./ v) ./ pdS - prm.cd0); % squared load factor
n(n < 1) = 1; % turn is impossible below n^2 < 1
% turn rate
omega = 9.81 ./ v .* sqrt(n - 1);
end