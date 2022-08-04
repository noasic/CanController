-------------------------------------------------------------------------------
--
-- CAN Controller Toplevel
--  
-- Part of CAN Controller project
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

use work.can_pkg.all;
use work.can_controller_pkg.all;

entity can_controller is
    port(
        i_clk          : in  std_logic; -- System clock, e.g. 100 MHz
        i_reset        : in  std_logic; -- System reset (synchronous, active-high)
        -- Configuration interface
        i_config       : in  t_can_controller_config;
        -- Tx-message interface
        i_tx_msg       : in  t_can_msg;
        i_tx_msg_valid : in  std_logic;
        o_tx_msg_ready : out std_logic;
        -- Rx-message interface
        o_rx_msg       : out t_can_msg;
        o_rx_msg_valid : out std_logic;
        -- Status interface
        o_status       : out t_can_controller_status;
        -- Interrupt interface
        o_interrupt    : out t_can_controller_interrupt;
        -- CAN PHY interface
        o_can_tx       : out std_logic;
        i_can_rx       : in  std_logic;
        -- Debug interface
        o_debug        : out t_can_controller_debug
    );
end entity can_controller;

architecture RTL of can_controller is
    signal s_quantum_ce      : std_logic;
    signal s_btl_can_tx      : std_logic;
    signal s_btl_can_rx      : std_logic;
    signal s_btl_hard_sync   : std_logic;
    signal s_btl_tx_clk      : std_logic;
    signal s_btl_rx_bit      : std_logic;
    signal s_bsp_tx_bit      : std_logic;
    signal s_bsp_bus_idle    : std_logic;
    signal s_bsp_stuff_error : std_logic;
    signal s_bsp_form_error  : std_logic;
    signal s_bsp_ack_error   : std_logic;
    signal s_bsp_bit_error   : std_logic;
    signal s_bsp_arb_loss    : std_logic;
    signal s_bsp_crc_error   : std_logic;
    signal s_can_tx          : std_logic;
    signal s_error_state     : unsigned(1 downto 0);
    signal s_btl_rx_clk      : std_logic;
    signal s_bsp_overload    : std_logic;
begin

    ----------------------------------------------------------------------------
    -- Bit Stream Processor (BSP)
    --
    bsp : entity work.can_bsp
        port map(
            i_clk                 => i_clk,
            i_reset               => i_reset,
            i_testmode            => i_config.testmode,
            i_user_tx_msg         => i_tx_msg,
            i_user_tx_msg_valid   => i_tx_msg_valid,
            o_user_tx_msg_ready   => o_tx_msg_ready,
            o_user_rx_msg         => o_rx_msg,
            o_user_rx_msg_valid   => o_rx_msg_valid,
            i_btl_hard_sync       => s_btl_hard_sync,
            i_btl_tx_clk          => s_btl_tx_clk,
            i_btl_rx_clk          => s_btl_rx_clk,
            i_btl_rx_bit          => s_btl_rx_bit,
            o_tx_bit              => s_bsp_tx_bit,
            o_bus_idle            => s_bsp_bus_idle,
            o_stuff_error         => s_bsp_stuff_error,
            o_form_error          => s_bsp_form_error,
            o_ack_error           => s_bsp_ack_error,
            o_bit_error           => s_bsp_bit_error,
            o_arb_loss            => s_bsp_arb_loss,
            o_crc_error           => s_bsp_crc_error,
            o_overload            => s_bsp_overload,
            o_error_state         => s_error_state,
            o_rx_error_count_u8   => o_status.rx_error_count,
            o_tx_error_count_u9   => o_status.tx_error_count,
            o_debug_can_field     => o_debug.bsp_can_field,
            o_debug_can_field_slv => o_debug.bsp_can_field_slv,
            o_debug_transmitting  => o_debug.bsp_transmitting,
            o_debug_receiving     => o_debug.bsp_receiving,
            o_debug_stuff_bit     => o_debug.bsp_stuff_bit,
            o_debug_bit_idx_u5    => o_debug.bsp_bit_idx_u5,
            o_debug_byte_idx_u3   => o_debug.bsp_byte_idx_u3
        );

    ----------------------------------------------------------------------------
    -- Bit Timing Logic
    --
    btl : entity work.can_btl
        port map(
            i_clk                  => i_clk,
            i_reset                => i_reset,
            i_tseg1_u4             => i_config.tseg1,
            i_tseg2_u3             => i_config.tseg2,
            i_sjw                  => i_config.sjw,
            i_quantum_ce           => s_quantum_ce,
            o_hard_sync            => s_btl_hard_sync,
            o_tx_clk               => s_btl_tx_clk,
            o_rx_clk               => s_btl_rx_clk,
            o_rx_bit               => s_btl_rx_bit,
            i_bsp_tx_bit           => s_bsp_tx_bit,
            i_bsp_bus_idle         => s_bsp_bus_idle,
            o_can_tx               => s_btl_can_tx,
            i_can_rx               => s_btl_can_rx,
            o_debug_tx_clk         => o_debug.btl_tx_clk,
            o_debug_rx_clk         => o_debug.btl_rx_clk,
            o_debug_hard_sync      => o_debug.btl_hard_sync,
            o_debug_resync         => o_debug.btl_resync,
            o_debug_phase_error_s6 => o_debug.btl_phase_error_s6
        );

    ----------------------------------------------------------------------------
    -- Prescaler
    --
    prescaler : entity work.can_prescaler
        port map(
            i_clk          => i_clk,
            i_reset        => i_reset,
            i_prescaler_u7 => i_config.prescaler,
            o_quantum_ce   => s_quantum_ce);

    ----------------------------------------------------------------------------
    -- Test Mode Multiplexers
    --
    s_can_tx     <= s_btl_can_tx when i_config.testmode = NORMAL_OPERATION else '1'; -- don't drive the bus in loopback and listen-only modes
    s_btl_can_rx <= s_btl_can_tx when i_config.testmode = LOOPBACK else i_can_rx;

    ----------------------------------------------------------------------------
    -- Outputs
    --
    o_can_tx                <= s_can_tx when s_error_state /= BUS_OFF else '1'; -- A 'bus off' unit is not allowed to have any influence on the bus. (E.g. output drivers switched off.) (CAN_CONTROLLER_REQ_FC3)
    o_debug.quant_clk       <= s_quantum_ce;
    o_debug.can_rx          <= i_can_rx;
    o_debug.can_tx          <= s_can_tx when s_error_state /= BUS_OFF else '1';
    o_interrupt.bit_error   <= s_bsp_bit_error;
    o_interrupt.stuff_error <= s_bsp_stuff_error;
    o_interrupt.crc_error   <= s_bsp_crc_error;
    o_interrupt.form_error  <= s_bsp_form_error;
    o_interrupt.ack_error   <= s_bsp_ack_error;
    o_interrupt.arb_loss    <= s_bsp_arb_loss;
    o_interrupt.overload    <= s_bsp_overload;
    o_status.error_state    <= s_error_state;

end architecture RTL;
