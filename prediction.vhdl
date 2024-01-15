------------------------------------------------------------------------------Title      : Prediction Unit for Pafui
-- Project    : 
-------------------------------------------------------------------------------
-- File       : prediction.vhd
-- Author     : Marcel Putsche  <...@hrz.tu-chemnitz.de>
-- Company    : TU-Chemmnitz, SSE
-- Created    : 2017-11-17
-- Last update: 2017-11-20
-- Platform   : x86_64-redhat-linux
-- Standard   : VHDL'87
-------------------------------------------------------------------------------
-- Description: Prediction Component for particle filter
-------------------------------------------------------------------------------
-- Copyright (c) 2017 TU-Chemmnitz, SSE
-------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author  Description
-- 2017-11-17  1.0      mput	Created

-- Group Members : Faizan Habib , Ali Abbas
---------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;
entity prediction is
port
(
clk        : in std_logic;                       -- Clock
rst        : in std_logic;                       -- High active reset
particle_i : in std_logic_vector (27 downto 0);  -- Particle input
valid_i    : in std_logic;                       -- Valid particle input

particle_o : out std_logic_vector (27 downto 0);  -- Particle output
valid_o    : out std_logic                        -- Valid particle output 
);
end prediction;

architecture RTL of prediction is
signal rnd_valid       : std_logic;      -- signal shall be '1' when both
-- random_gen deliver valid values
signal particle_x      : std_logic_vector (13 downto 0);
signal particle_y      : std_logic_vector (13 downto 0);
signal rnd_x, rnd_y    : signed (15 downto 0);
signal pred_x_temp     : signed (15 downto 0);
signal pred_y_temp     : signed (15 downto 0);
signal pred_temp_valid : std_logic;

signal pred_x     : unsigned (13 downto 0);
signal pred_y     : unsigned (13 downto 0);
signal pred_valid : std_logic;

constant l1_cn : natural := 16;  -- natural value 1 of calcualtion border
constant l2_cn : natural := 16;  -- natural value 2 of calcualtion border

constant l1_cu : unsigned(13 downto 0) := to_unsigned(l1_cn, 14);  -- unsigned
-- value 1 of calcualtion border
constant l2_cu : unsigned(13 downto 0) := to_unsigned(l2_cn, 14);  -- unsigend
-- value 2 of calcualtion border

component random_gen
generic (
seed : unsigned(31 downto 0);
m    : natural;
a    : unsigned(31 downto 0);
c    : unsigned(31 downto 0));
port (
clk         : in  std_logic;
rst         : in  std_logic;
rnd_min_max : in  unsigned(15 downto 0);
rnd_out     : out signed(15 downto 0);
rnd_valid   : out std_logic
);
end component;
begin
-------------------------------------------------------------------------------
--Instances for the random generator
-----------------------------------------------------------------------------
random_gen_x : random_gen
generic map (
seed => to_unsigned(0, 32),
m    => 32,
a    => to_unsigned(1664525, 32),
c    => to_unsigned(1013904223, 32))
port map (
clk =>clk,
rst =>rst,
rnd_min_max => (to_unsigned(20, 16)),
rnd_out =>rnd_x,
rnd_valid =>rnd_valid);

random_gen_y : random_gen
generic map (
seed => to_unsigned(0, 32),
m    => 32,
a    => to_unsigned(1025925, 32),
c    => to_unsigned(1096704223, 32))
port map (
clk =>clk,
rst =>rst,
rnd_min_max => (to_unsigned(20, 16)),
rnd_out =>rnd_y,
rnd_valid =>rnd_valid);

-------------------------------------------------------------
-- SPLIT PARTICLE
-- process to split up the values of the incoming particles
-- shall be synchronous with an asynchronous reset
-- the incoming particles shall only be processed when valid_i
-- is '1'
-- Input: clk, rst, particle_i, valid_i
-- Output: particle_x, particle_y
-------------------------------------------------------------

SPLIT_I : process (clk, rst)
begin
if rst='1' then
particle_x <=(others=>'0');
particle_y <=(others=>'0');


elsif clk'event AND clk ='1' then

if valid_i='1' then
particle_x <=particle_i(27 downto 14);
particle_y <=particle_i(13 downto 0);
end if;
end if;
end process;
-------------------------------------------------------------
-- APPLY MOTION MODEL
-- process to add the random values from the random_gen to
-- x and y values and store them as intermediate values
-- shall be synchronous with an asynchronous reset
-- Input: clk, rst,rnd_valid, valid_i, particle_x, particle_y
-- Output: pred_x_temp, pred_y_temp, pred_temp_valid
-------------------------------------------------------------

APPL_MOTION : process (clk, rst)
begin
if rst='1' then
pred_x_temp <= (others=>'0');
pred_y_temp <= (others=>'0');
pred_temp_valid <='0';

elsif clk'event AND clk ='1' then

if valid_i = '1' then
if rnd_valid = '1' then
pred_x_temp <= signed(particle_x)+rnd_x;
pred_y_temp <= signed(particle_y)+rnd_y;
pred_temp_valid <='1';
end if;
end if;
end if;
end process;
-------------------------------------------------------------
-- CHECK AGAINST BORDERS
-- process to check whether the intermediate values are inside 
-- the predefined borders l1_cn and l2_cn
-- if not: if the value is larger then the given border, the value shall be
-- set to the max border value. if the value is smaller than 0, the value
-- shall be set to 0
-- shall be synchronous with an asynchronous reset
-- Input: clk, rst,pred_x_temp, pred_y_temp, pred_temp_valid
-- Output: pred_x, pred_y, pred_valid
-------------------------------------------------------------
CHECK : process (clk, rst)
begin
if (rst='1') then
pred_x <= (others=>'0');
pred_y <= (others=>'0'); 
pred_valid <='0';

elsif (clk'event AND clk ='1') then
if (pred_temp_valid='1') then
pred_x<= resize(unsigned(pred_x_temp),pred_x'length);
pred_y<= resize(unsigned(pred_y_temp),pred_y'length);

--    pred_x<= unsigned(std_logic_vector(pred_x_temp(13 downto 0)));
--   pred_y<= unsigned(std_logic_vector(pred_y_temp(13 downto 0)));

if unsigned(pred_x_temp(13 downto 0)) > (l1_cu) then
pred_x <= l1_cu;
end if;

if unsigned(pred_y_temp(13 downto 0)) > (l1_cu) then
pred_y <= l1_cu;
end if;

if to_integer(pred_x_temp) < 0 then
pred_x <= (others => '0');
end if;

if to_integer(pred_y_temp) < 0 then
pred_y <= (others => '0');
end if;
pred_valid <= '1';  
end if;
end if;
end process;
-------------------------------------------------------------------------------
--internl logic
------------------------------------------------------------------------
-------------------------------------------------------------
-- MERGE COORDINATES
-- the separated particles shall be merged again to output particle_o
-------------------------------------------------------------
particle_o(27 downto 14) <= std_logic_vector(unsigned(pred_x));
particle_o(13 downto 0) <= std_logic_vector(unsigned(pred_y));
valid_o<= pred_valid;

end architecture RTL