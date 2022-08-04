-------------------------------------------------------------------------------
--
-- Prescaler
--  
-- Part of CAN Controller project
--
-- Tasks:
--   * Generate quantum clock-enable 
--  
-- Author(s):
--   Guy Eschemann, guy@noasic.com
--  
-------------------------------------------------------------------------------
--
-- Copyright (c) 2022 Guy Eschemann
-- 
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
-- 
--     http://www.apache.org/licenses/LICENSE-2.0
-- 
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity can_prescaler is
    port(
        i_clk          : in  std_logic;
        i_reset        : in  std_logic;
        -- Configuration interface
        i_prescaler_u7 : in  unsigned(7 downto 0); -- length of a time quantum (TQ) in system clock cycles. actual length is value + 1
        -- BTL interface
        o_quantum_ce   : out std_logic
    );
end entity can_prescaler;

architecture RTL of can_prescaler is

    -- Registered signals
    signal s_quantum_ce_r : std_logic := '0';

begin

    ----------------------------------------------------------------------------
    -- Prescaler process
    --
    prescaler : process(i_clk) is
        variable v_count_u7_r : unsigned(7 downto 0) := (others => '0');
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                v_count_u7_r   := (others => '0');
                s_quantum_ce_r <= '0';
            else
                -- defaults:
                s_quantum_ce_r <= '0';

                if v_count_u7_r = i_prescaler_u7 then
                    s_quantum_ce_r <= '1';
                    v_count_u7_r   := (others => '0');
                else
                    v_count_u7_r := v_count_u7_r + 1;
                end if;
            end if;
        end if;
    end process prescaler;

    ----------------------------------------------------------------------------
    -- Outputs
    --
    o_quantum_ce <= s_quantum_ce_r;

end architecture RTL;
