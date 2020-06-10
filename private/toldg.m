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

function res = toldg(prm, res)
%% Take off and langing performance
% Compute the take-off and landing performance at sea-level, based on
% Gudmunsson formula
% Adrien Crovato

% constants
w = prm.mtow * 9.81; % weight
s = prm.S; % wing area
cd0 = prm.cd0; % zero-lift drag
k = 1 / (prm.eps * prm.ar * pi); % induced drag factor

hO = 50 * 0.3048; % obstacle height
muTo = 0.03; % free-roll friction coefficient (concrete)
vLof = 1.1 * res.vs;
vTr = 1.2 * res.vs;
clTo = 0.5 * prm.clmax;
muLd = 0.3; % braking friction coefficient (concrete)
pLd = 1.6; % relative increase in CL_max at landing (due to high-lift device)
vTd = 1.1 * res.vs / sqrt(pLd); % touch-down speed
clLd = 0.75 * prm.clmax * pLd;

% take-off
pdSTo = 0.5 * 1.225 * vLof^2 / 2 * s;
sG = vLof^2 * w / (2 * 9.81 * (thrust(vLof/sqrt(2), 0, prm) - pdSTo * (cd0 + k * clTo^2) - muTo * (w - pdSTo * clTo))); % ground run
if (prm.mtow < 25000); sR = vLof; else; sR = 3 * vLof; end % rotation
rTr = 0.2156 * res.vs^2;
clTr = 2 * w / (1.225 * vTr^2 * s);
sThetaTr = thrust(vTr, 0, prm) / w - cd0 / clTr + k * clTr;
hTr = rTr * (1 - sqrt(1 - sThetaTr^2));
if hTr < hO
    sO = rTr * sThetaTr + (hO - hTr) / sThetaTr * sqrt(1 - sThetaTr^2); % transition
else
    sO = sqrt(rTr^2 - (rTr - hO)^2); % transition
end
sTO = sG + sR + sO; % take-off distance

% landing
pdSLd = 0.5 * 1.225 * vTd^2 / 2 * s;
sA = (hO - 0.1512 * res.vs^2 / pLd * (1 - cosd(3))) / tand(3); % final approach
sF = res.vs^2 / pLd * sind(3); % flare
if (prm.mtow < 25000); sFR = vTd; else; sFR = 3 * vTd; end % free roll
sBR = - vTd^2 * w / (2 * 9.81 * (0.05 * thrust(0, 0, prm) - pdSLd * (cd0 + k * clLd) - muLd * (w - pdSLd * clLd))); % ground run
sLD = sA + sF + sFR + sBR;

% Assign
res.tor = sG;
res.tod = sTO;
res.ldr = sFR + sBR;
res.ldd = sLD;

% Display
disp(['Take-off run = ', num2str(round(sG / 0.3048)), ' ft']);
disp(['Take-off distance = ', num2str(round(sTO / 0.3048)), ' ft']);
disp(['Landing run = ', num2str(round((sFR+sBR) / 0.3048)), ' ft']);
disp(['Landing distance = ', num2str(round(sLD / 0.3048)), ' ft']);
end
