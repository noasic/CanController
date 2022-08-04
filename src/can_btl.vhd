-------------------------------------------------------------------------------
--
-- CAN Controller Bit Timing Logic (BTL)
--  
-- Part of CAN Controller project
--
-- Description:
--   * Handles bit-level synchronization to the CAN bus
--   * Transmits single bits on the CAN bus
--   * Receives (samples) single bits from the CAN bus
--   * Generates sampling clock for the BSP module
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

use work.can_controller_pkg.all;

entity can_btl is
    port(
        i_clk                  : in  std_logic; -- System clock, e.g. 100 MHz
        i_reset                : in  std_logic; -- System reset (synchronous, active-high)
        -- Configuration interface
        i_tseg1_u4             : in  unsigned(3 downto 0); -- Time segment 1. Length of the first time segment in time quantums. Range: [1..15]
        i_tseg2_u3             : in  unsigned(2 downto 0); -- Time segment 1. Length of the second time segment in time quantums. Range: [0..7]
        i_sjw                  : in  unsigned(1 downto 0); -- Synchronization jump width. Range: [0..3] (CAN_CONTROLLER_REQ_BT7)
        -- Prescaler interface
        i_quantum_ce           : in  std_logic; -- Quantum clock-enable (1 pulse per time quanta) from prescaler
        -- BSP interface
        o_hard_sync            : out std_logic;
        o_tx_clk               : out std_logic;
        o_rx_clk               : out std_logic;
        o_rx_bit               : out std_logic;
        i_bsp_tx_bit           : in  std_logic;
        i_bsp_bus_idle         : in  std_logic;
        -- CAN PHY interface
        o_can_tx               : out std_logic;
        i_can_rx               : in  std_logic;
        -- Debug interface
        o_debug_tx_clk         : out std_logic;
        o_debug_rx_clk         : out std_logic;
        o_debug_hard_sync      : out std_logic;
        o_debug_resync         : out std_logic;
        o_debug_phase_error_s6 : out std_logic_vector(5 downto 0)
    );
end entity can_btl;

architecture RTL of can_btl is

    -- Returns the minimum of two signed vectors
    function min(a : signed; b : signed) return signed is
    begin
        if a <= b then
            return a;
        else
            return b;
        end if;
    end function;

    -- Returns the maximum of two signed vectors
    function max(a : signed; b : signed) return signed is
    begin
        if a >= b then
            return a;
        else
            return b;
        end if;
    end function;

    -- Types:
    type t_state is (SYNC_SEG, TSEG_1, TSEG_2);

    -- Unregistered signals:
    signal s_can_rx_sync      : std_logic; -- CAN Rx-signal, synchronized to system clock
    signal s_can_rx_edge_edge : std_logic; -- Detected a R -> D edge on the CAN Rx-line 

    -- Registered signals:
    signal s_state_r                : t_state            := SYNC_SEG;
    signal s_quant_count_s6_r       : signed(5 downto 0) := (others => '0'); -- p.67 number of TQ in a bit time has to be programmable at least from 8 to 25
    signal s_sample_bit_value_r     : std_logic          := '1'; -- Sampled bit value
    signal s_sample_bit_value_old_r : std_logic          := '1'; -- Previous sampled bit value
    signal s_phase_error_s6_r       : signed(5 downto 0) := (others => '0'); -- Phase error of an edge. Range: [-8..16]
    signal s_do_resync_r            : std_logic          := '0'; -- Resynchronization request
    signal s_lock_sync_r            : std_logic          := '0'; -- Avoid multiple synchronizations during the same bit time
    signal s_tx_clk_r               : std_logic          := '0';
    signal s_rx_clk_r               : std_logic          := '0';
    signal s_debug_tx_clk_r         : std_logic          := '0'; -- Stretched TX-clock pulse, for debug purposes
    signal s_debug_rx_clk_r         : std_logic          := '0'; -- Stretched RX-clock pulse, for debug purposes
    signal s_hard_sync_r            : std_logic          := '0';
    signal s_debug_hard_sync_r      : std_logic          := '0'; -- Stretched hard-sync pulse, for debug purposes

begin

    -------------------------------------------------------------------------------
    -- CAN Rx-Signal Synchronizer
    --
    can_rx_synchronizer : entity work.synchronizer
        generic map(
            G_INIT_VALUE    => '1',
            G_NUM_GUARD_FFS => 1)
        port map(
            i_reset => i_reset,
            i_clk   => i_clk,
            i_data  => i_can_rx,
            o_data  => s_can_rx_sync);

    -------------------------------------------------------------------------------
    -- Detect R->D edges on CAN RX-signal
    --
    r_to_d_edge_detector : entity work.edge_detector
        generic map(
            G_EDGE_TYPE  => "BOTH",
            G_INIT_LEVEL => '1')
        port map(
            i_clk   => i_clk,
            i_reset => i_reset,
            i_ce    => i_quantum_ce,
            i_data  => s_can_rx_sync,
            o_edge  => s_can_rx_edge_edge);

    -------------------------------------------------------------------------------
    -- BTL State Machine (runs on quantum clock)
    --
    btl_fsm : process(i_clk) is
        variable v_tseg1_s6_r : signed(5 downto 0) := (others => '0'); -- actual length of TSEG1, possibly corrected for resynchronization. Range: [1..16], corrected: [1..20]
        variable v_tseg2_s6_r : signed(5 downto 0) := (others => '0'); -- actual length of TSEG2, possibly corrected for resynchronization. Range: [1..8]
        variable v_sjw_s6     : signed(5 downto 0);
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                v_tseg1_s6_r         := (others => '0');
                v_tseg2_s6_r         := (others => '0');
                s_debug_hard_sync_r  <= '0';
                s_hard_sync_r        <= '0';
                s_sample_bit_value_r <= '1';
                s_quant_count_s6_r   <= (others => '0');
                s_do_resync_r        <= '0';
                s_phase_error_s6_r   <= (others => '0');
                s_lock_sync_r        <= '0';
                s_tx_clk_r           <= '0';
                s_rx_clk_r           <= '0';
                s_debug_tx_clk_r     <= '0';
                s_debug_rx_clk_r     <= '0';
                s_hard_sync_r        <= '0';
                s_state_r            <= SYNC_SEG;
            else
                -- defaults: 
                s_tx_clk_r    <= '0';
                s_rx_clk_r    <= '0';
                s_hard_sync_r <= '0';

                if i_quantum_ce = '1' then
                    -- default:
                    s_debug_tx_clk_r    <= '0'; -- these debug clocks stretch over one bit time for better visibility on logic analyzers
                    s_debug_rx_clk_r    <= '0';
                    s_debug_hard_sync_r <= '0';

                    case s_state_r is

                        -- Synchronization segment
                        --   * length is fixed 1 TQ
                        --   * restart quant counter
                        --
                        when SYNC_SEG =>
                            -- Latch timing parameters, so they remain constant for the whole during of the bit time
                            -- Using 6-bit signed integers to avoid lots of type casts & resizing
                            v_tseg1_s6_r := signed("0" & resize(i_tseg1_u4, 5) + 1); -- actual length is tseg1 + 1
                            v_tseg2_s6_r := signed("00" & resize(i_tseg2_u3, 4) + 1); -- actual length is tseg2 + 1
                            v_sjw_s6     := signed("000" & resize(i_sjw, 3) + 1); -- actual jump width is sjw + 1

                            -- Check ranges of timing parameters
                            -- pragma translate_off
                            assert v_tseg1_s6_r >= 2 and v_tseg2_s6_r <= 16 severity failure;
                            assert v_tseg2_s6_r >= 1 and v_tseg2_s6_r <= 8 severity failure;
                            assert v_sjw_s6 >= 1 and v_sjw_s6 <= 4 severity failure;
                            -- pragma translate_on

                            -- Correct timing parameters in case of resynchronization
                            if s_do_resync_r = '1' then
                                s_lock_sync_r <= '1'; -- CAN_CONTROLLER_REQ_BT1
                                s_do_resync_r <= '0';
                                --
                                if s_phase_error_s6_r > 0 then -- need to lengthen TSEG1 by a maximum of the synchronization jump width
                                    v_tseg1_s6_r := v_tseg1_s6_r + min(v_sjw_s6, s_phase_error_s6_r); -- limit of lengthening is the synchronization jump width (CAN_CONTROLLER_REQ_BT5, CAN_CONTROLLER_REQ_BT9)
                                else    -- need to shorten TSEG2 by a maximum of the synchronization jump width
                                    v_tseg2_s6_r := v_tseg2_s6_r + max(-v_sjw_s6, s_phase_error_s6_r); -- limit of shortening is the synchronization jump width (CAN_CONTROLLER_REQ_BT5, CAN_CONTROLLER_REQ_BT10)
                                end if;
                            end if;

                            s_quant_count_s6_r <= s_quant_count_s6_r + 1;
                            s_state_r          <= TSEG_1;

                        -- Time segment 1
                        --   * length: 2..16 TQs
                        --
                        when TSEG_1 =>
                            -- Check whether resynchronization is required
                            --   * allow only one synchronization within one bit time (CAN_CONTROLLER_REQ_BT1)
                            --   * only recessive to dominant edges qualify (CAN_CONTROLLER_REQ_BT2, CAN_CONTROLLER_REQ_BT4)
                            --   * do not resynchronize in case of positive phase error when transmitting a dominant bit (CAN_CONTROLLER_REQ_BT4a) 
                            if s_lock_sync_r = '0' and s_can_rx_edge_edge = '1' and s_can_rx_sync = '0' and s_sample_bit_value_r = '1' and i_bsp_tx_bit /= '0' then
                                s_phase_error_s6_r <= s_quant_count_s6_r; -- CAN_CONTROLLER_REQ_BT6
                                s_do_resync_r      <= '1';
                            end if;
                            --
                            s_quant_count_s6_r <= s_quant_count_s6_r + 1;
                            --
                            if s_quant_count_s6_r = v_tseg1_s6_r then -- end of TSEG1 is sample time
                                s_rx_clk_r               <= '1';
                                s_debug_rx_clk_r         <= '1';
                                s_sample_bit_value_r     <= s_can_rx_sync;
                                s_sample_bit_value_old_r <= s_sample_bit_value_r;
                                s_state_r                <= TSEG_2;
                            end if;

                        -- Time segment 2
                        --   * length: 1..8 TQs
                        --
                        when TSEG_2 =>
                            -- Check whether resynchronization is required
                            --   * allow only one synchronization within one bit time (CAN_CONTROLLER_REQ_BT1)
                            --   * only recessive to dominant edges qualify (CAN_CONTROLLER_REQ_BT2, CAN_CONTROLLER_REQ_BT4) 
                            if s_lock_sync_r = '0' and s_can_rx_edge_edge = '1' and s_can_rx_sync = '0' and s_sample_bit_value_r = '1' then
                                s_phase_error_s6_r <= s_quant_count_s6_r - (v_tseg1_s6_r + v_tseg2_s6_r); -- CAN_CONTROLLER_REQ_BT6
                                s_do_resync_r      <= '1';
                            end if;
                            --
                            s_quant_count_s6_r <= s_quant_count_s6_r + 1;
                            --
                            if s_quant_count_s6_r = v_tseg1_s6_r + v_tseg2_s6_r then
                                s_quant_count_s6_r <= (others => '0');
                                s_lock_sync_r      <= '0';
                                s_tx_clk_r         <= '1';
                                s_debug_tx_clk_r   <= '1';
                                s_state_r          <= SYNC_SEG;
                            end if;

                    end case;

                end if;                 -- if i_quantum_ce = '1'

                -- Hard synchronization
                --   * performed whenever there is a recessive to dominant edge during bus idle (CAN_CONTROLLER_REQ_BT3)
                --   * only if bus value differs from value at previous sample point (CAN_CONTROLLER_REQ_BT2)
                --   * only one synchronization within one bit time is allowed (CAN_CONTROLLER_REQ_BT1)
                --   * after a hard synchronization, the internal bit time is restarted with SYNC_SEG (CAN_CONTROLLER_REQ_BT8)
                --
                if i_bsp_bus_idle = '1' and s_can_rx_sync = '0' and s_sample_bit_value_old_r = '1' and s_hard_sync_r = '0' then
                    -- pragma translate_off
                    assert s_do_resync_r = '0' severity failure; -- should be mutually exclusive
                    -- pragma translate_on
                    s_do_resync_r       <= '0'; -- hard synchronization has priority over resynchronization (CAN_CONTROLLER_REQ_BT4)
                    s_lock_sync_r       <= '1'; -- lock synchronization for the duration of the bit time (CAN_CONTROLLER_REQ_BT1)
                    s_quant_count_s6_r  <= (others => '0'); -- CAN_CONTROLLER_REQ_BT8
                    s_hard_sync_r       <= '1';
                    s_debug_hard_sync_r <= '1';
                    s_tx_clk_r          <= '1';
                    s_debug_tx_clk_r    <= '1';
                    s_state_r           <= SYNC_SEG; -- CAN_CONTROLLER_REQ_BT8
                end if;

            end if;                     -- i_reset = '1'
        end if;                         -- rising_edge(i_clk)
    end process btl_fsm;

    -------------------------------------------------------------------------------
    -- Outputs
    --
    o_rx_bit               <= s_sample_bit_value_r;
    o_tx_clk               <= s_tx_clk_r;
    o_rx_clk               <= s_rx_clk_r;
    o_hard_sync            <= s_hard_sync_r;
    o_can_tx               <= i_bsp_tx_bit;
    o_debug_tx_clk         <= s_debug_tx_clk_r;
    o_debug_rx_clk         <= s_debug_rx_clk_r;
    o_debug_hard_sync      <= s_debug_hard_sync_r;
    o_debug_resync         <= s_do_resync_r;
    o_debug_phase_error_s6 <= std_logic_vector(s_phase_error_s6_r);

end architecture RTL;
