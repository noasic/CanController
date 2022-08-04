-------------------------------------------------------------------------------
--
-- CAN Controller Bit Stream Processor (BSP)
--  
-- Part of CAN Controller project
--
-- Tasks:
--   * Serialize/deserialize messages into/from bits
--   * Insert/remove stuff bits
--   * Compute CRC
--   * Monitor received bits during transmission and react accordingly
--   * Bus arbitration
--   * Retry on lost arbitration
--   * Detect FORM, CRC, ACK, STUFF and BIT errors
--   * Manage error state and RX/TX error counters
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

entity can_bsp is
    port(
        i_clk                 : in  std_logic; -- System clock, e.g. 100 MHz
        i_reset               : in  std_logic; -- System reset (synchronous, active-high)
        -- Configuration interface
        i_testmode            : in  unsigned(1 downto 0); -- Test mode (0: normal operation, 1: listen-only mode, 2: loopback mode)        
        -- Tx-message interface
        i_user_tx_msg         : in  t_can_msg;
        i_user_tx_msg_valid   : in  std_logic;
        o_user_tx_msg_ready   : out std_logic;
        -- Rx-message interface
        o_user_rx_msg         : out t_can_msg;
        o_user_rx_msg_valid   : out std_logic;
        -- BTL interface
        i_btl_hard_sync       : in  std_logic;
        i_btl_tx_clk          : in  std_logic; -- transmit (i.e. start-of-frame) clock
        i_btl_rx_clk          : in  std_logic; -- receive (i.e. sample) clock
        i_btl_rx_bit          : in  std_logic;
        o_tx_bit              : out std_logic;
        o_bus_idle            : out std_logic; -- enables hard synchronization in BTL
        -- Interrupt outputs
        o_stuff_error         : out std_logic; -- STUFF error
        o_form_error          : out std_logic; -- FORM error while receiving
        o_ack_error           : out std_logic; -- ACK error while transmitting
        o_bit_error           : out std_logic; -- BIT ERROR while transmitting
        o_arb_loss            : out std_logic; -- lost arbitration while transmitting
        o_crc_error           : out std_logic; -- CRC error while receiving
        o_overload            : out std_logic; -- detected an overload condition
        -- Status interface
        o_error_state         : out unsigned(1 downto 0); -- The CAN controller's error state (0: error active, 1: error passive, 2: bus off)
        o_rx_error_count_u8   : out unsigned(7 downto 0); -- The receive error counter
        o_tx_error_count_u9   : out unsigned(8 downto 0); -- The transmit error counter
        -- Debug interface
        o_debug_can_field     : out t_can_field; -- the CAN message field currently on the bus
        o_debug_can_field_slv : out std_logic_vector(4 downto 0); -- std_logic_vector version of 'o_debug_can_field'
        o_debug_transmitting  : out std_logic; -- whether the CAN controller is currently transmitting
        o_debug_receiving     : out std_logic; -- whether the CAN controller is currently transmitting
        o_debug_stuff_bit     : out std_logic; -- whether the current bit on the CAN bus is a stuff bit
        o_debug_bit_idx_u5    : out std_logic_vector(4 downto 0); -- bit index within the current field (unsigned integer)
        o_debug_byte_idx_u3   : out std_logic_vector(2 downto 0) -- index of the current data field byte (unsigned integer)
    );
end entity can_bsp;

architecture RTL of can_bsp is

    -- Constants
    constant EXTENDED_ID_LENGTH_BITS          : natural                                        := 18; -- CAN_CONTROLLER_REQ_DF4
    constant CRC_LENGTH_BITS                  : natural                                        := 15;
    constant BASE_ID_LENGTH_BITS              : natural                                        := 11;
    constant SYNC_LENGTH_BITS                 : natural                                        := 11; -- CAN_CONTROLLER_REQ_BAS3
    constant EOF_LENGTH_BITS                  : natural                                        := 7; -- CAN_CONTROLLER_REQ_DF10
    constant DLC_LENGTH_BITS                  : natural                                        := 4; -- CAN_CONTROLLER_REQ_DF3
    constant MAX_IDENTICAL_BITS               : natural                                        := 5; -- CAN_CONTROLLER_REQ_COD1a
    constant INTERMISSION_LENGTH_BITS         : natural                                        := 3; -- CAN_CONTROLLER_REQ_IF3
    constant SUSPEND_TRANSMISSION_LENGTH_BITS : natural                                        := 8; -- CAN_CONTROLLER_REQ_IF5
    constant ERROR_FLAG_LENGTH_BITS           : natural                                        := 6; -- CAN_CONTROLLER_REQ_EF1a, CAN_CONTROLLER_REQ_EF2b
    constant ERROR_DELIMITER_LENGTH_BITS      : natural                                        := 8; -- CAN_CONTROLLER_REQ_EF3
    constant OVERLOAD_FLAG_LENGTH_BITS        : natural                                        := 6; -- CAN_CONTROLLER_REQ_OV1
    constant OVERLOAD_DELIMITER_LENGTH_BITS   : natural                                        := 8; -- CAN_CONTROLLER_REQ_OV2
    constant ERROR_PASSIVE_LEVEL              : natural                                        := 128; -- CAN_CONTROLLER_REQ_FC4l
    constant BUS_OFF_LEVEL                    : natural                                        := 256; -- CAN_CONTROLLER_REQ_FC4n
    constant CRC_INIT                         : std_logic_vector(CRC_LENGTH_BITS - 1 downto 0) := (others => '0');

    -- Compute the updated CRC given the current CRC and the bit value (CAN_CONTROLLER_REQ_DF23)
    impure function update_crc(bit_value : std_logic; crc : std_logic_vector) return std_logic_vector is
        variable v_crcnxt : std_logic;
        variable v_crc    : std_logic_vector(crc'range);
    begin
        v_crc              := crc;
        v_crcnxt           := bit_value xor v_crc(14);
        v_crc(14 downto 1) := v_crc(13 downto 0);
        v_crc(0)           := '0';
        if v_crcnxt = '1' then
            v_crc(14 downto 0) := v_crc(14 downto 0) xor "100010110011001"; -- 0x4599
        end if;
        return v_crc;
    end function;

    -- Types
    type t_error_flag_cause is (NONE, STUFF_ERROR, FORM_ERROR, ACK_ERROR, BIT_ERROR, CRC_ERROR);

    -- Registered signals
    signal s_can_field_r           : t_can_field                                                                  := BUS_IDLE;
    signal s_can_field_old_r       : t_can_field                                                                  := BUS_IDLE;
    signal s_transmitting_r        : std_logic                                                                    := '0';
    signal s_receiving_r           : std_logic                                                                    := '0';
    signal s_bit_idx_r             : natural                                                                      := 0; -- index of current bit within the current field
    signal s_byte_idx_r            : natural range 0 to 7                                                         := 0; -- index of current byte in the DATA field
    signal s_rx_msg_r              : t_can_msg                                                                    := ((others => '0'), '0', '0', (others => '0'), (others => '0'));
    signal s_rx_msg_received_crc_r : std_logic_vector(CRC_LENGTH_BITS - 1 downto 0)                               := (others => '0');
    signal s_rx_msg_computed_crc_r : std_logic_vector(CRC_LENGTH_BITS - 1 downto 0)                               := (others => '0');
    signal s_crc_r                 : std_logic_vector(CRC_LENGTH_BITS - 1 downto 0)                               := (others => '0');
    signal s_tx_msg_crc_shreg_r    : std_logic_vector(CRC_LENGTH_BITS - 1 downto 0)                               := (others => '0');
    signal s_rx_msg_valid_r        : std_logic                                                                    := '0';
    signal s_tx_msg_id_shreg_r     : std_logic_vector(EXTENDED_ID_LENGTH_BITS + BASE_ID_LENGTH_BITS - 1 downto 0) := (others => '0'); -- TX message ID shift register
    signal s_tx_stuff_value_r      : std_logic                                                                    := '0'; -- the value of the transmitted stuff bit
    signal s_tx_msg_dlc_shreg_r    : std_logic_vector(DLC_LENGTH_BITS - 1 downto 0)                               := (others => '0'); -- TX message DLC shift register
    signal s_tx_msg_data_shreg_r   : std_logic_vector(63 downto 0)                                                := (others => '0'); -- TX message DATA shift register
    signal s_tx_msg_ready_r        : std_logic                                                                    := '0';
    signal s_ack_error_r           : std_logic                                                                    := '0';
    signal s_bit_error_r           : std_logic                                                                    := '0';
    signal s_form_error_r          : std_logic                                                                    := '0';
    signal s_crc_error_r           : std_logic                                                                    := '0';
    signal s_overload_r            : std_logic                                                                    := '0';
    signal s_testmode_r            : unsigned(1 downto 0)                                                         := (others => '0');
    signal s_arb_loss_r            : std_logic                                                                    := '0';
    signal s_stuff_error_r         : std_logic                                                                    := '0';
    signal s_rx_error_count_u8_r   : unsigned(7 downto 0)                                                         := (others => '0'); -- receive error count (CAN_CONTROLLER_REQ_FC4)
    signal s_tx_error_count_u9_r   : unsigned(8 downto 0)                                                         := (others => '0'); -- transmit error count (CAN_CONTROLLER_REQ_FC4)
    signal s_rx_is_stuff_bit_r     : std_logic                                                                    := '0'; -- signals that the received bit is a STUFF bit
    signal s_tx_stuff_bit_r        : std_logic                                                                    := '0'; -- signals that the next bit to be transmitted is a STUFF bit
    signal s_rx_stuff_error_r      : std_logic                                                                    := '0';
    signal s_is_startup_r          : std_logic                                                                    := '1'; -- signals the start-up phase, during which repeated ACK errors do not lead to BUS OFF
    signal s_is_stuff_bit_r        : std_logic                                                                    := '0'; -- signals that the current bit is a STUFF bit
    signal s_tx_rx_bit_mismatch_r  : std_logic                                                                    := '0';

    -- Unregistered signals
    signal s_btl_tx_bit  : std_logic;
    signal s_error_state : unsigned(1 downto 0);
    signal s_stuff_en    : std_logic;
    signal s_crc_en      : std_logic;

begin

    -------------------------------------------------------------------------------
    -- BSP FSM 
    --  
    --  * Messages whose transmission was aborted because of an error are 
    --    re-transmitted automatically because o_user_tx_msg_ready is not asserted 
    --    until the message has been transmitted successfully (CAN_CONTROLLER_REQ_BAS5) 
    -- 
    bsp_fsm_seq : process(i_clk) is
        -- State variables:
        variable v_temp_bit_count_r             : natural range 0 to 8 := 0;
        variable v_error_flag_bit_polarity_r    : std_logic            := '1';
        variable v_dominant_bit_detected_r      : boolean              := false; -- signals that a dominant bit was detected while sending a PASSIVE ERROR FLAG
        variable v_error_flag_cause_r           : t_error_flag_cause   := NONE;
        variable v_arbitration_tx_stuff_error_r : boolean              := false; -- signals a stuff error detected during the transmission of ARBITRATION, in which a D has been monitored while expecting a R (to handle CAN_CONTROLLER_REQ_FC4e) 
        variable v_rx_msg_ide_r                 : std_logic            := '0';

        -- Helper variables:
        variable v_rx_error_count_u8 : unsigned(7 downto 0);
        variable v_tx_error_count_u9 : unsigned(8 downto 0);
        variable v_rx_dlc            : unsigned(3 downto 0);
        variable v_error             : boolean;

        -- Increase the transmit error count by the given value 
        procedure increase_tx_error_count(value : positive) is
        begin
            -- pragma translate_off
            assert s_transmitting_r = '1' report "unexpected receive error count change" severity failure;
            -- pragma translate_on
            v_tx_error_count_u9 := v_tx_error_count_u9 + value;
        end procedure;

        -- Increase the receive error count by the given value 
        procedure increase_rx_error_count(value : positive) is
        begin
            -- pragma translate_off
            assert s_receiving_r = '1' and s_transmitting_r = '0' report "unexpected receive error count change" severity failure;
            -- pragma translate_on
            if v_rx_error_count_u8 < ERROR_PASSIVE_LEVEL then -- The Receive Error Count needs not to be incremented above the Error Passive level (CAN_CONTROLLER_REQ_ADD1)
                v_rx_error_count_u8 := v_rx_error_count_u8 + value;
            end if;
        end procedure;

        -- Decrease the transmit error count by the given value 
        procedure decrease_tx_error_count(value : positive) is
        begin
            -- pragma translate_off
            assert s_transmitting_r = '1' report "unexpected receive error count change" severity failure;
            assert value = 1 severity failure;
            -- pragma translate_on
            if v_tx_error_count_u9 > 0 then
                v_tx_error_count_u9 := v_tx_error_count_u9 - value;
            end if;
        end procedure;

        -- Decrease the receive error count by the given value 
        procedure decrease_rx_error_count(value : positive) is
        begin
            -- pragma translate_off
            assert s_receiving_r = '1' and s_transmitting_r = '0' report "unexpected receive error count change" severity failure;
            assert value = 1 severity failure;
            -- pragma translate_on
            if v_rx_error_count_u8 > 127 then
                v_rx_error_count_u8 := to_unsigned(127, v_rx_error_count_u8'length); -- CAN_CONTROLLER_REQ_FC4k
            elsif v_rx_error_count_u8 > 0 then
                v_rx_error_count_u8 := v_rx_error_count_u8 - value;
            end if;
        end procedure;

        -- Returns the type error flag to transmit (active or passive), depending on the 
        -- RX and TX error counters and the current error state.
        impure function error_flag return t_can_field is
            variable v_result : t_can_field;
        begin
            if v_rx_error_count_u8 >= ERROR_PASSIVE_LEVEL or v_tx_error_count_u9 >= ERROR_PASSIVE_LEVEL then
                v_result := PASSIVE_ERROR_FLAG; -- CAN_CONTROLLER_REQ_ERR6b, CAN_CONTROLLER_REQ_EF2, CAN_CONTROLLER_REQ_FC2
                if s_error_state = ERROR_ACTIVE then
                    v_result := ACTIVE_ERROR_FLAG; -- An error condition letting a node become 'error passive' causes the node to send an ACTIVE ERROR FLAG (CAN_CONTROLLER_REQ_FC4m)
                end if;
            else
                v_result := ACTIVE_ERROR_FLAG; -- CAN_CONTROLLER_REQ_ERR6a, CAN_CONTROLLER_REQ_EF1, CAN_CONTROLLER_REQ_FC1
            end if;
            return v_result;
        end function;

    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                v_temp_bit_count_r             := 0;
                v_error_flag_bit_polarity_r    := '1';
                v_dominant_bit_detected_r      := false;
                v_error_flag_cause_r           := NONE;
                v_arbitration_tx_stuff_error_r := false;
                v_rx_msg_ide_r                 := '0';
                s_transmitting_r               <= '0';
                s_receiving_r                  <= '0';
                s_byte_idx_r                   <= 0;
                s_bit_idx_r                    <= 0;
                s_rx_msg_valid_r               <= '0';
                s_rx_msg_r                     <= ((others => '0'), '0', '0', (others => '0'), (others => '0'));
                s_rx_msg_computed_crc_r        <= (others => '0');
                s_rx_msg_received_crc_r        <= (others => '0');
                s_tx_msg_id_shreg_r            <= (others => '0');
                s_tx_msg_dlc_shreg_r           <= (others => '0');
                s_tx_msg_ready_r               <= '0';
                s_ack_error_r                  <= '0';
                s_form_error_r                 <= '0';
                s_crc_error_r                  <= '0';
                s_overload_r                   <= '0';
                s_bit_error_r                  <= '0';
                s_arb_loss_r                   <= '0';
                s_stuff_error_r                <= '0';
                s_rx_error_count_u8_r          <= (others => '0');
                s_tx_error_count_u9_r          <= (others => '0');
                s_is_startup_r                 <= '1';
                s_tx_msg_crc_shreg_r           <= (others => '0');
                s_can_field_r                  <= SYNCHRONIZE;
                s_can_field_old_r              <= SYNCHRONIZE;
            else
                -- defaults:
                s_rx_msg_valid_r <= '0';
                s_tx_msg_ready_r <= '0';
                s_ack_error_r    <= '0';
                s_form_error_r   <= '0';
                s_crc_error_r    <= '0';
                s_overload_r     <= '0';
                s_bit_error_r    <= '0';
                s_arb_loss_r     <= '0';
                s_stuff_error_r  <= '0';

                if i_btl_tx_clk = '1' then
                    -- defaults:
                    v_rx_error_count_u8 := s_rx_error_count_u8_r;
                    v_tx_error_count_u9 := s_tx_error_count_u9_r;

                    -- check that bit/byte counters are reset when changing states:
                    -- pragma translate_off
                    if s_can_field_r /= s_can_field_old_r then
                        assert s_bit_idx_r = 0 report "bit index must be reset when changing states" severity failure;
                        assert s_byte_idx_r = 0 report "byte index must be reset when changing states" severity failure;
                    end if;
                    -- pragma translate_on

                    case s_can_field_r is

                        --------------------------------------------------------                        
                        -- Synchronize to the bus activity by waiting for eleven 
                        -- consecutive recessive bits (CAN_CONTROLLER_REQ_BAS3),
                        -- or remain in this state until reset in case of BUS OFF.
                        --
                        when SYNCHRONIZE =>
                            if i_btl_rx_bit = '1' then
                                if s_bit_idx_r = SYNC_LENGTH_BITS - 1 then
                                    s_bit_idx_r   <= 0;
                                    s_can_field_r <= BUS_IDLE;
                                else
                                    s_bit_idx_r <= s_bit_idx_r + 1;
                                end if;
                            else
                                s_bit_idx_r <= 0;
                            end if;

                        --------------------------------------------------------
                        -- BUS IDLE
                        -- 
                        when BUS_IDLE =>
                            -- pragma translate_off
                            assert s_rx_is_stuff_bit_r = '0' severity failure;
                            assert s_tx_stuff_bit_r = '0' severity failure;
                            -- pragma translate_on
                            s_transmitting_r               <= '0'; -- The unit stays TRANSMITTER until the bus is idle or the unit loses ARBITRATION. (CAN_CONTROLLER_REQ_DEF1)
                            s_receiving_r                  <= '0'; -- A unit is called "RECEIVER" of a message, if it is not TRANSMITTER of that message and the bus is not idle. (CAN_CONTROLLER_REQ_DEF2)
                            s_bit_idx_r                    <= 0;
                            s_testmode_r                   <= i_testmode; -- hold testmode constant for the full message duration
                            v_dominant_bit_detected_r      := false;
                            v_error_flag_cause_r           := NONE;
                            v_arbitration_tx_stuff_error_r := false;
                            --
                            if i_btl_hard_sync = '1' then -- hard sync., i.e. the detection of a dominant bit, signals start of reception (CAN_CONTROLLER_REQ_IF4b)
                                s_receiving_r <= '1'; -- A unit is called "RECEIVER" of a message, if it is not TRANSMITTER of that message and the bus is not idle. (CAN_CONTROLLER_REQ_DEF2)
                                s_can_field_r <= SOF;
                            end if;
                            --
                            if i_user_tx_msg_valid = '1' then -- When the bus is free any unit may start to transmit a message (CAN_CONTROLLER_REQ_BAS2)
                                s_transmitting_r <= '1'; -- A unit originating a message is called "TRANSMITTER" of that message. (CAN_CONTROLLER_REQ_DEF1)
                                s_receiving_r    <= '1'; -- always enable receiving when transmitting, for the case that we lose arbitration
                                s_can_field_r    <= SOF;
                            end if;

                        --------------------------------------------------------
                        -- Start of frame
                        --
                        when SOF =>
                            -- pragma translate_off
                            assert v_dominant_bit_detected_r = false severity failure;
                            assert v_error_flag_cause_r = NONE severity failure;
                            assert s_receiving_r = '1' severity failure; -- always receiving, because arbitration not completed yet
                            assert s_rx_is_stuff_bit_r = '0' severity failure;
                            assert s_tx_stuff_bit_r = '0' severity failure;
                            assert v_arbitration_tx_stuff_error_r = false severity failure;
                            -- pragma translate_on

                            -- RX logic
                            -- --------
                            s_rx_msg_r <= ((others => '0'), '0', '0', (others => '0'), (others => '0'));

                            -- TX logic
                            -- --------
                            if s_transmitting_r = '1' then
                                s_tx_msg_id_shreg_r <= std_logic_vector(i_user_tx_msg.id);
                            end if;

                            -- Next state logic
                            -- ----------------
                            s_bit_idx_r   <= 0;
                            s_can_field_r <= ID11;

                            -- Error handling
                            -- --------------
                            if s_transmitting_r = '1' and s_tx_rx_bit_mismatch_r = '1' then
                                v_error_flag_cause_r := BIT_ERROR;
                                s_bit_error_r        <= '1'; -- BIT ERROR (CAN_CONTROLLER_REQ_ERR4)
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;

                        ---------------------------------------------------------
                        -- ARBITRATION FIELD, 11-bit identifier
                        --
                        --   * In Standard Format the ARBITRATION FIELD consists of the 11 bit IDENTIFIER and the RTR-BIT (CAN_CONTROLLER_REQ_DF17)
                        --   * In Extended Format the ARBITRATION FIELD consists of the 29 bit IDENTIFIER, the SRR-Bit, the IDE-Bit, and the RTR-BIT (CAN_CONTROLLER_REQ_DF18) 
                        --   * In an Extended Frame the Base ID is transmitted first, followed by the SRR bit and the IDE bit. The Extended ID is transmitted after the IDE bit. (CAN_CONTROLLER_REQ_DF20)
                        --
                        when ID11 =>
                            -- pragma translate_off
                            assert s_receiving_r = '1' severity failure; -- always receiving, because arbitration not completed yet
                            -- pragma translate_on

                            -- RX logic
                            -- --------
                            if s_rx_is_stuff_bit_r = '0' then
                                s_rx_msg_r.id(28 downto 18) <= s_rx_msg_r.id(27 downto 18) & i_btl_rx_bit; -- CAN_CONTROLLER_REQ_DF2
                            end if;

                            -- TX logic
                            -- --------
                            if s_transmitting_r = '1' and s_tx_stuff_bit_r = '0' then
                                s_tx_msg_id_shreg_r <= s_tx_msg_id_shreg_r(s_tx_msg_id_shreg_r'high - 1 downto 0) & '0';
                            end if;

                            -- Next state logic
                            -- ----------------
                            if s_tx_stuff_bit_r = '0' then
                                if s_bit_idx_r = BASE_ID_LENGTH_BITS - 1 then
                                    s_bit_idx_r   <= 0;
                                    s_can_field_r <= RTR_OR_SRR; -- CAN_CONTROLLER_REQ_DF19
                                else
                                    s_bit_idx_r <= s_bit_idx_r + 1;
                                end if;
                            end if;

                            -- Error handling
                            -- --------------
                            -- Lost arbitration detection (only after transmitting a non-stuff bit): 
                            if s_transmitting_r = '1' and s_rx_is_stuff_bit_r = '0' and s_tx_rx_bit_mismatch_r = '1' and i_btl_rx_bit = '0' then -- no BIT error when transmitting a dominant bit in arbitration field (CAN_CONTROLLER_REQ_ERR4a)
                                s_transmitting_r <= '0'; -- The unit stays TRANSMITTER until the bus is idle or the unit loses ARBITRATION. (CAN_CONTROLLER_REQ_DEF1)
                                s_arb_loss_r     <= '1'; -- CAN_CONTROLLER_REQ_BAS4
                            end if;
                            v_error := false;
                            if s_transmitting_r = '1' and s_tx_rx_bit_mismatch_r = '1' and i_btl_rx_bit = '1' then -- TX, sent '0', received '1' -> BIT ERROR
                                v_error              := true;
                                v_error_flag_cause_r := BIT_ERROR;
                                s_bit_error_r        <= '1'; -- BIT ERROR (CAN_CONTROLLER_REQ_ERR4)
                            end if;
                            if s_rx_stuff_error_r = '1' then
                                v_error                        := true;
                                v_arbitration_tx_stuff_error_r := false;
                                if s_transmitting_r = '1' and i_btl_rx_bit = '0' then
                                    v_arbitration_tx_stuff_error_r := true; -- CAN_CONTROLLER_REQ_FC4e
                                end if;
                                if s_receiving_r = '1' and s_transmitting_r = '0' then
                                    increase_rx_error_count(1); -- increase received error count by 1 (CAN_CONTROLLER_REQ_FC4a)
                                end if;
                                v_error_flag_cause_r := STUFF_ERROR;
                                s_stuff_error_r      <= '1';
                            end if;
                            if v_error then
                                s_bit_idx_r   <= 0;
                                s_can_field_r <= ERROR_FLAG;
                            end if;

                        --------------------------------------------------------
                        -- ARBITRATION FIELD, RTR (standard format) or SRR (extended format)
                        --
                        --   * In Standard Format the ARBITRATION FIELD consists of the 11 bit IDENTIFIER and the RTR-BIT (CAN_CONTROLLER_REQ_DF17)
                        --   * In Extended Format the ARBITRATION FIELD consists of the 29 bit IDENTIFIER, the SRR-Bit, the IDE-Bit, and the RTR-BIT (CAN_CONTROLLER_REQ_DF18)                        
                        --   * In an Extended Frame the Base ID is transmitted first, followed by the SRR bit and the IDE bit. The Extended ID is transmitted after the IDE bit. (CAN_CONTROLLER_REQ_DF20)
                        --                        
                        when RTR_OR_SRR =>
                            -- RX logic
                            -- --------
                            if s_receiving_r = '1' and s_rx_is_stuff_bit_r = '0' then
                                s_rx_msg_r.rtr <= i_btl_rx_bit;
                            end if;

                            -- TX logic
                            -- --------
                            -- In standard frames, arbitration can be declared "won" at the end of the RTR field
                            if s_transmitting_r = '1' and i_user_tx_msg.ide = '0' and -- transmitting a standard frame 
                            s_rx_is_stuff_bit_r = '0' and i_btl_rx_bit = s_btl_tx_bit and -- RTR bit was not overwritten
                            s_testmode_r /= LOOPBACK then
                                s_receiving_r <= '0';
                            end if;

                            -- Next state logic
                            -- ----------------
                            if s_tx_stuff_bit_r = '0' then
                                s_bit_idx_r   <= 0;
                                s_can_field_r <= IDE;
                            end if;

                            -- Error handling
                            -- --------------
                            if s_transmitting_r = '1' and s_tx_rx_bit_mismatch_r = '1' then
                                if i_btl_rx_bit = '0' and s_rx_is_stuff_bit_r = '0' then
                                    -- Lost arbitration, no BIT ERROR when transmitting a dominant bit in arbitration field (CAN_CONTROLLER_REQ_ERR4a)
                                    s_transmitting_r <= '0'; -- The unit stays TRANSMITTER until the bus is idle or the unit loses ARBITRATION. (CAN_CONTROLLER_REQ_DEF1)
                                    s_arb_loss_r     <= '1'; -- CAN_CONTROLLER_REQ_BAS4
                                else
                                    -- BIT ERROR while transmitting
                                    s_bit_error_r        <= '1'; -- BIT ERROR (CAN_CONTROLLER_REQ_ERR4)
                                    v_error_flag_cause_r := BIT_ERROR;
                                    s_bit_idx_r          <= 0;
                                    s_can_field_r        <= ERROR_FLAG;
                                end if;
                            end if;
                            if s_rx_stuff_error_r = '1' then
                                if s_receiving_r = '1' and s_transmitting_r = '0' then
                                    increase_rx_error_count(1); -- increase received error count by 1 (CAN_CONTROLLER_REQ_FC4a)
                                end if;
                                v_arbitration_tx_stuff_error_r := false;
                                if s_transmitting_r = '1' and i_btl_rx_bit = '0' then
                                    v_arbitration_tx_stuff_error_r := true; -- CAN_CONTROLLER_REQ_FC4e
                                end if;
                                s_stuff_error_r      <= '1';
                                v_error_flag_cause_r := STUFF_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;

                        --------------------------------------------------------
                        -- CONTROL FIELD, IDE (standard format) or ARBITRATION FIELD, SRR (extended format)
                        --
                        --   * In Extended Format the ARBITRATION FIELD consists of the 29 bit IDENTIFIER, the SRR-Bit, the IDE-Bit, and the RTR-BIT (CAN_CONTROLLER_REQ_DF18)
                        --   * In an Extended Frame the Base ID is transmitted first, followed by the SRR bit and the IDE bit. The Extended ID is transmitted after the IDE bit. (CAN_CONTROLLER_REQ_DF20)
                        --
                        when IDE =>

                            -- RX logic
                            -- --------
                            if s_receiving_r = '1' and s_rx_is_stuff_bit_r = '0' then
                                v_rx_msg_ide_r := i_btl_rx_bit;
                                s_rx_msg_r.ide <= v_rx_msg_ide_r;
                            end if;

                            -- Next state logic
                            -- ----------------                                                        
                            if s_tx_stuff_bit_r = '0' then
                                if (s_transmitting_r = '1' and i_user_tx_msg.ide = '0') or --
                                (s_receiving_r = '1' and v_rx_msg_ide_r = '0') then -- standard format
                                    s_bit_idx_r   <= 0;
                                    s_can_field_r <= R0;
                                else    -- extended format          
                                    s_bit_idx_r   <= 0;
                                    s_can_field_r <= ID18;
                                end if;
                            end if;

                            -- Error handling
                            -- --------------                            
                            if s_transmitting_r = '1' and s_tx_rx_bit_mismatch_r = '1' then
                                -- There are two cases where transmitting a '1' and receiving a '0' signal a lost arbitration:
                                --   * if this a stuff bit for the RTR field in a standard frame
                                --   * in extended frames where IDE is part of the arbitration field                                
                                if i_btl_rx_bit = '0' and s_rx_is_stuff_bit_r = '0' and i_user_tx_msg.ide = '1' then
                                    s_transmitting_r <= '0'; -- The unit stays TRANSMITTER until the bus is idle or the unit loses ARBITRATION. (CAN_CONTROLLER_REQ_DEF1)
                                    s_arb_loss_r     <= '1'; -- lost arbitration, no BIT ERROR when transmitting a dominant bit in arbitration field (CAN_CONTROLLER_REQ_ERR4a, CAN_CONTROLLER_REQ_BAS4
                                else
                                    -- BIT ERROR while transmitting
                                    s_bit_error_r        <= '1'; -- BIT ERROR (CAN_CONTROLLER_REQ_ERR4)
                                    v_error_flag_cause_r := BIT_ERROR;
                                    s_bit_idx_r          <= 0;
                                    s_can_field_r        <= ERROR_FLAG;
                                end if;
                            end if;
                            if s_rx_stuff_error_r = '1' then
                                if s_receiving_r = '1' and s_transmitting_r = '0' then
                                    increase_rx_error_count(1); -- increase received error count by 1 (CAN_CONTROLLER_REQ_FC4a)
                                end if;
                                v_arbitration_tx_stuff_error_r := false;
                                if s_transmitting_r = '1' and i_btl_rx_bit = '0' then
                                    v_arbitration_tx_stuff_error_r := true; -- CAN_CONTROLLER_REQ_FC4e
                                end if;
                                s_stuff_error_r      <= '1';
                                v_error_flag_cause_r := STUFF_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;

                        --------------------------------------------------------
                        -- ARBITRATION FIELD, 18-bit identifier
                        --
                        --   * In Extended Format the ARBITRATION FIELD consists of the 29 bit IDENTIFIER, the SRR-Bit, the IDE-Bit, and the RTR-BIT (CAN_CONTROLLER_REQ_DF18)
                        --   * In an Extended Frame the Base ID is transmitted first, followed by the SRR bit and the IDE bit. The Extended ID is transmitted after the IDE bit. (CAN_CONTROLLER_REQ_DF20)
                        --                        
                        when ID18 =>
                            -- pragma translate_off                            
                            assert s_receiving_r = '1' severity failure; -- always receiving, because arbitration not completed yet
                            -- pragma translate_on

                            -- RX logic
                            -- --------
                            if s_rx_is_stuff_bit_r = '0' then
                                s_rx_msg_r.id(17 downto 0) <= s_rx_msg_r.id(16 downto 0) & i_btl_rx_bit; -- extended ID is transmitted MSB-first (CAN_CONTROLLER_REQ_DF4)
                            end if;

                            -- TX logic
                            -- --------
                            if s_transmitting_r = '1' and s_tx_stuff_bit_r = '0' then
                                s_tx_msg_id_shreg_r <= s_tx_msg_id_shreg_r(s_tx_msg_id_shreg_r'high - 1 downto 0) & '0'; -- extended ID is transmitted MSB-first (CAN_CONTROLLER_REQ_DF4)
                            end if;

                            -- Next state logic
                            -- ----------------
                            if s_tx_stuff_bit_r = '0' then
                                if s_bit_idx_r = EXTENDED_ID_LENGTH_BITS - 1 then
                                    s_bit_idx_r   <= 0;
                                    s_can_field_r <= RTR_EXT;
                                else
                                    s_bit_idx_r <= s_bit_idx_r + 1;
                                end if;
                            end if;

                            -- Error handling
                            -- --------------
                            if s_transmitting_r = '1' and s_tx_rx_bit_mismatch_r = '1' then
                                if i_btl_rx_bit = '0' and s_rx_is_stuff_bit_r = '0' then
                                    -- Lost arbitration, no BIT ERROR when transmitting a dominant bit in arbitration field (CAN_CONTROLLER_REQ_ERR4a)
                                    s_transmitting_r <= '0'; -- The unit stays TRANSMITTER until the bus is idle or the unit loses ARBITRATION. (CAN_CONTROLLER_REQ_DEF1)
                                    s_arb_loss_r     <= '1'; -- CAN_CONTROLLER_REQ_BAS4
                                else
                                    -- BIT ERROR while transmitting
                                    s_bit_error_r        <= '1'; -- BIT ERROR (CAN_CONTROLLER_REQ_ERR4)
                                    v_error_flag_cause_r := BIT_ERROR;
                                    s_bit_idx_r          <= 0;
                                    s_can_field_r        <= ERROR_FLAG;
                                end if;
                            end if;
                            if s_rx_stuff_error_r = '1' then
                                if s_receiving_r = '1' and s_transmitting_r = '0' then
                                    increase_rx_error_count(1); -- increase received error count by 1 (CAN_CONTROLLER_REQ_FC4a)
                                end if;
                                v_arbitration_tx_stuff_error_r := false;
                                if s_transmitting_r = '1' and i_btl_rx_bit = '0' then
                                    v_arbitration_tx_stuff_error_r := true; -- CAN_CONTROLLER_REQ_FC4e
                                end if;
                                s_stuff_error_r      <= '1';
                                v_error_flag_cause_r := STUFF_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;

                        --------------------------------------------------------
                        -- ARBITRATION FIELD, RTR (extended format)
                        --
                        --   * In Extended Format the ARBITRATION FIELD consists of the 29 bit IDENTIFIER, the SRR-Bit, the IDE-Bit, and the RTR-BIT (CAN_CONTROLLER_REQ_DF18)
                        --                        
                        when RTR_EXT =>
                            -- pragma translate_off
                            assert s_receiving_r = '1' severity failure; -- always receiving, because arbitration not completed yet
                            -- pragma translate_on     

                            -- RX logic
                            -- --------
                            if s_rx_is_stuff_bit_r = '0' then
                                s_rx_msg_r.rtr <= i_btl_rx_bit;
                            end if;

                            -- Next state logic
                            -- ----------------
                            if s_tx_stuff_bit_r = '0' then
                                if s_transmitting_r = '1' and s_receiving_r = '1' and s_testmode_r = NORMAL_OPERATION then
                                    -- Arbitration won -> stop reception (except in test modes)
                                    s_receiving_r <= '0';
                                end if;
                                s_bit_idx_r   <= 0;
                                s_can_field_r <= R1;
                            end if;

                            -- Error handling
                            -- --------------
                            if s_transmitting_r = '1' and s_tx_rx_bit_mismatch_r = '1' then
                                if i_btl_rx_bit = '0' and s_rx_is_stuff_bit_r = '0' then
                                    -- Lost arbitration, no BIT ERROR when transmitting a dominant bit in arbitration field (CAN_CONTROLLER_REQ_ERR4a)
                                    s_transmitting_r <= '0'; -- The unit stays TRANSMITTER until the bus is idle or the unit loses ARBITRATION. (CAN_CONTROLLER_REQ_DEF1)
                                    s_arb_loss_r     <= '1'; -- CAN_CONTROLLER_REQ_BAS4
                                else
                                    -- BIT ERROR while transmitting
                                    s_bit_error_r        <= '1'; -- BIT ERROR (CAN_CONTROLLER_REQ_ERR4)
                                    v_error_flag_cause_r := BIT_ERROR;
                                    s_bit_idx_r          <= 0;
                                    s_can_field_r        <= ERROR_FLAG;
                                end if;
                            end if;
                            if s_rx_stuff_error_r = '1' then
                                if s_receiving_r = '1' and s_transmitting_r = '0' then
                                    increase_rx_error_count(1); -- increase received error count by 1 (CAN_CONTROLLER_REQ_FC4a)
                                end if;
                                v_arbitration_tx_stuff_error_r := false;
                                if s_transmitting_r = '1' and i_btl_rx_bit = '0' then
                                    v_arbitration_tx_stuff_error_r := true; -- CAN_CONTROLLER_REQ_FC4e
                                end if;
                                s_stuff_error_r      <= '1';
                                v_error_flag_cause_r := STUFF_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;

                        --------------------------------------------------------
                        -- CONTROL FIELD, R1 (extended format)
                        --
                        when R1 =>
                            -- RX logic
                            -- --------                            
                            -- For reserved bits, receivers accept 'dominant' and 'recessive' bits in all combinations (CAN_CONTROLLER_REQ_DF16)

                            -- Next state logic
                            -- ----------------                            
                            if s_tx_stuff_bit_r = '0' then
                                s_bit_idx_r   <= 0;
                                s_can_field_r <= R0;
                            end if;

                            -- Error handling
                            -- --------------
                            if s_transmitting_r = '1' and s_tx_rx_bit_mismatch_r = '1' then -- bit error
                                -- BIT ERROR while transmitting
                                s_bit_error_r        <= '1'; -- BIT ERROR (CAN_CONTROLLER_REQ_ERR4)
                                v_error_flag_cause_r := BIT_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;
                            if s_rx_stuff_error_r = '1' then
                                if s_receiving_r = '1' and s_transmitting_r = '0' then
                                    increase_rx_error_count(1); -- increase received error count by 1 (CAN_CONTROLLER_REQ_FC4a)
                                end if;
                                s_stuff_error_r      <= '1';
                                v_error_flag_cause_r := STUFF_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;

                        --------------------------------------------------------
                        -- CONTROL FIELD, R0
                        --
                        when R0 =>

                            -- RX logic
                            -- --------                            
                            -- For reserved bits, receivers accept 'dominant' and 'recessive' bits in all combinations (CAN_CONTROLLER_REQ_DF16)

                            -- Next state logic
                            -- ----------------                            
                            if s_tx_stuff_bit_r = '0' then
                                s_tx_msg_dlc_shreg_r <= std_logic_vector(i_user_tx_msg.dlc); -- preload DLC shift register
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= DLC;
                            end if;

                            -- Error handling
                            -- --------------
                            if s_transmitting_r = '1' and s_tx_rx_bit_mismatch_r = '1' then
                                -- BIT ERROR while transmitting
                                s_bit_error_r        <= '1'; -- BIT ERROR (CAN_CONTROLLER_REQ_ERR4)
                                v_error_flag_cause_r := BIT_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;
                            if s_rx_stuff_error_r = '1' then
                                if s_receiving_r = '1' and s_transmitting_r = '0' then
                                    increase_rx_error_count(1); -- increase received error count by 1 (CAN_CONTROLLER_REQ_FC4a)
                                end if;
                                s_stuff_error_r      <= '1';
                                v_error_flag_cause_r := STUFF_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;

                        --------------------------------------------------------
                        -- CONTROL FIELD, DLC
                        --
                        --   * DATA FRAME: admissible numbers of data bytes: {0,1,....,7,8}. Other values may not be used. (CAN_CONTROLLER_REQ_DF21)
                        --
                        when DLC =>
                            -- RX logic
                            -- --------                        
                            if s_receiving_r = '1' and s_rx_is_stuff_bit_r = '0' then
                                v_rx_dlc       := s_rx_msg_r.dlc(2 downto 0) & i_btl_rx_bit;
                                s_rx_msg_r.dlc <= v_rx_dlc;
                            end if;

                            -- TX logic
                            -- --------                            
                            if s_transmitting_r = '1' and s_tx_stuff_bit_r = '0' then
                                s_tx_msg_dlc_shreg_r <= s_tx_msg_dlc_shreg_r(s_tx_msg_dlc_shreg_r'high - 1 downto 0) & '0';
                            end if;

                            -- Next state logic
                            -- ----------------
                            if s_tx_stuff_bit_r = '0' then
                                if s_bit_idx_r = DLC_LENGTH_BITS - 1 then
                                    s_bit_idx_r <= 0;
                                    if s_transmitting_r = '1' then
                                        if i_user_tx_msg.dlc = 0 or i_user_tx_msg.rtr = '1' then
                                            s_tx_msg_crc_shreg_r    <= s_crc_r; -- initialize CRC TX shift register before entering CRC_SEQ
                                            s_rx_msg_computed_crc_r <= s_crc_r; -- latch computed CRC before entering CRC_SEQ
                                            s_can_field_r           <= CRC_SEQ;
                                        else
                                            s_byte_idx_r          <= 0;
                                            s_tx_msg_data_shreg_r <= i_user_tx_msg.data;
                                            s_can_field_r         <= DATA;
                                        end if;
                                    else -- receiving
                                        if v_rx_dlc = 0 or s_rx_msg_r.rtr = '1' then -- no DATA FIELD in remote frames (CAN_CONTROLLER_REQ_RF1)
                                            s_tx_msg_crc_shreg_r    <= s_crc_r; -- initialize CRC TX shift register before entering CRC_SEQ
                                            s_rx_msg_computed_crc_r <= s_crc_r; -- latch computed CRC before entering CRC_SEQ
                                            s_can_field_r           <= CRC_SEQ;
                                        else
                                            if v_rx_dlc > 8 then
                                                s_rx_msg_r.dlc <= to_unsigned(8, s_rx_msg_r.dlc'length); -- CAN_CONTROLLER_REQ_ADD3
                                            end if;
                                            s_byte_idx_r  <= 0;
                                            s_can_field_r <= DATA;
                                        end if;
                                    end if;
                                else
                                    s_bit_idx_r <= s_bit_idx_r + 1;
                                end if;
                            end if;

                            -- Error handling
                            -- --------------
                            if s_transmitting_r = '1' and s_tx_rx_bit_mismatch_r = '1' then
                                -- BIT ERROR while transmitting
                                s_bit_error_r        <= '1'; -- BIT ERROR (CAN_CONTROLLER_REQ_ERR4)
                                v_error_flag_cause_r := BIT_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;
                            if s_rx_stuff_error_r = '1' then
                                if s_receiving_r = '1' and s_transmitting_r = '0' then
                                    increase_rx_error_count(1); -- increase received error count by 1 (CAN_CONTROLLER_REQ_FC4a)
                                end if;
                                s_stuff_error_r      <= '1';
                                v_error_flag_cause_r := STUFF_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;

                        --------------------------------------------------------
                        -- DATA FIELD
                        --
                        --   * The DATA FIELD consists of the data to be transferred within a DATA FRAME. It can contain from 0 to 8 bytes, which each contain 8 bits which are transferred MSB first. (CAN_CONTROLLER_REQ_DF22)
                        --
                        when DATA =>
                            -- RX logic
                            -- --------
                            if s_receiving_r = '1' then
                                if s_rx_is_stuff_bit_r = '0' then
                                    if s_byte_idx_r = 0 then
                                        s_rx_msg_r.data(63 downto 56) <= s_rx_msg_r.data(63 - 1 downto 56) & i_btl_rx_bit;
                                    elsif s_byte_idx_r = 1 then
                                        s_rx_msg_r.data(55 downto 48) <= s_rx_msg_r.data(55 - 1 downto 48) & i_btl_rx_bit;
                                    elsif s_byte_idx_r = 2 then
                                        s_rx_msg_r.data(47 downto 40) <= s_rx_msg_r.data(47 - 1 downto 40) & i_btl_rx_bit;
                                    elsif s_byte_idx_r = 3 then
                                        s_rx_msg_r.data(39 downto 32) <= s_rx_msg_r.data(39 - 1 downto 32) & i_btl_rx_bit;
                                    elsif s_byte_idx_r = 4 then
                                        s_rx_msg_r.data(31 downto 24) <= s_rx_msg_r.data(31 - 1 downto 24) & i_btl_rx_bit;
                                    elsif s_byte_idx_r = 5 then
                                        s_rx_msg_r.data(23 downto 16) <= s_rx_msg_r.data(23 - 1 downto 16) & i_btl_rx_bit;
                                    elsif s_byte_idx_r = 6 then
                                        s_rx_msg_r.data(15 downto 8) <= s_rx_msg_r.data(15 - 1 downto 8) & i_btl_rx_bit;
                                    else
                                        -- pragma translate_off
                                        assert s_byte_idx_r = 7 severity failure;
                                        -- pragma translate_on
                                        s_rx_msg_r.data(7 downto 0) <= s_rx_msg_r.data(7 - 1 downto 0) & i_btl_rx_bit;
                                    end if;
                                end if;
                            end if;

                            -- TX logic
                            -- --------
                            if s_transmitting_r = '1' and s_tx_stuff_bit_r = '0' then
                                s_tx_msg_data_shreg_r <= s_tx_msg_data_shreg_r(s_tx_msg_data_shreg_r'high - 1 downto 0) & '0';
                            end if;

                            -- Next state logic
                            -- ----------------
                            if s_tx_stuff_bit_r = '0' then
                                if s_bit_idx_r = 7 then
                                    s_bit_idx_r <= 0;
                                    --
                                    if s_transmitting_r = '1' then
                                        if s_byte_idx_r = i_user_tx_msg.dlc - 1 then
                                            s_byte_idx_r            <= 0;
                                            s_tx_msg_crc_shreg_r    <= s_crc_r; -- initialize CRC TX shift register before entering CRC_SEQ
                                            s_rx_msg_computed_crc_r <= s_crc_r; -- latch computed CRC before entering CRC_SEQ
                                            s_can_field_r           <= CRC_SEQ;
                                        else
                                            s_byte_idx_r <= s_byte_idx_r + 1;
                                        end if;
                                    else -- receiving
                                        if s_byte_idx_r = s_rx_msg_r.dlc - 1 then
                                            s_byte_idx_r            <= 0;
                                            s_tx_msg_crc_shreg_r    <= s_crc_r; -- initialize CRC TX shift register before entering CRC_SEQ
                                            s_rx_msg_computed_crc_r <= s_crc_r; -- latch computed CRC before entering CRC_SEQ
                                            s_can_field_r           <= CRC_SEQ;
                                        else
                                            s_byte_idx_r <= s_byte_idx_r + 1;
                                        end if;
                                    end if;
                                else
                                    s_bit_idx_r <= s_bit_idx_r + 1;
                                end if;
                            end if;

                            -- Error handling
                            -- --------------
                            if s_transmitting_r = '1' and s_tx_rx_bit_mismatch_r = '1' then
                                -- BIT ERROR while transmitting
                                s_bit_error_r        <= '1'; -- BIT ERROR (CAN_CONTROLLER_REQ_ERR4)
                                v_error_flag_cause_r := BIT_ERROR;
                                s_bit_idx_r          <= 0;
                                s_byte_idx_r         <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;
                            if s_rx_stuff_error_r = '1' then
                                if s_receiving_r = '1' and s_transmitting_r = '0' then
                                    increase_rx_error_count(1); -- increase received error count by 1 (CAN_CONTROLLER_REQ_FC4a)
                                end if;
                                s_stuff_error_r      <= '1';
                                v_error_flag_cause_r := STUFF_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;

                        --------------------------------------------------------
                        -- CRC FIELD, CRC sequence
                        --
                        when CRC_SEQ =>
                            -- RX logic
                            -- --------                        
                            if s_receiving_r = '1' and s_rx_is_stuff_bit_r = '0' then
                                s_rx_msg_received_crc_r <= s_rx_msg_received_crc_r(13 downto 0) & i_btl_rx_bit;
                            end if;

                            -- TX logic
                            -- --------                        
                            if s_transmitting_r = '1' and s_tx_stuff_bit_r = '0' then
                                s_tx_msg_crc_shreg_r <= s_tx_msg_crc_shreg_r(s_tx_msg_crc_shreg_r'high - 1 downto 0) & '0';
                            end if;

                            -- Next state logic
                            -- ----------------
                            if s_tx_stuff_bit_r = '0' then
                                if s_bit_idx_r = CRC_LENGTH_BITS - 1 then
                                    s_bit_idx_r   <= 0;
                                    s_can_field_r <= CRC_DELIMITER; -- The CRC SEQUENCE is followed by the CRC DELIMITER (CAN_CONTROLLER_REQ_DF5)
                                else
                                    s_bit_idx_r <= s_bit_idx_r + 1;
                                end if;
                            end if;

                            -- Error handling
                            -- --------------
                            if s_transmitting_r = '1' and s_tx_rx_bit_mismatch_r = '1' then
                                -- BIT ERROR while transmitting
                                s_bit_error_r        <= '1'; -- BIT ERROR (CAN_CONTROLLER_REQ_ERR4)
                                v_error_flag_cause_r := BIT_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;
                            if s_rx_stuff_error_r = '1' then
                                if s_receiving_r = '1' and s_transmitting_r = '0' then
                                    increase_rx_error_count(1); -- increase received error count by 1 (CAN_CONTROLLER_REQ_FC4a)
                                end if;
                                s_stuff_error_r      <= '1';
                                v_error_flag_cause_r := STUFF_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;

                        --------------------------------------------------------
                        -- CRC FIELD, CRC delimiter
                        --
                        when CRC_DELIMITER =>
                            -- pragma translate_off
                            assert s_rx_is_stuff_bit_r = '0' severity failure;
                            assert s_tx_stuff_bit_r = '0' severity failure;
                            assert s_rx_stuff_error_r = '0' severity failure;
                            -- pragma translate_on  

                            -- Next state logic
                            -- ----------------
                            if s_receiving_r = '1' and i_btl_rx_bit /= '1' then
                                s_form_error_r       <= '1'; -- a FORM ERROR has to be detected when a fixed-form bit field contains one or more illegal bits. (CAN_CONTROLLER_REQ_ERR2)
                                v_error_flag_cause_r := FORM_ERROR;
                                increase_rx_error_count(1); -- increase received error count by 1 (CAN_CONTROLLER_REQ_FC4a)
                                s_bit_idx_r   <= 0;
                                s_can_field_r <= ERROR_FLAG;
                            else
                                s_bit_idx_r   <= 0;
                                s_can_field_r <= ACK_SLOT;
                            end if;

                            -- Error handling
                            -- --------------
                            if s_transmitting_r = '1' and s_tx_rx_bit_mismatch_r = '1' then
                                -- BIT ERROR while transmitting
                                s_bit_error_r        <= '1'; -- BIT ERROR (CAN_CONTROLLER_REQ_ERR4)
                                v_error_flag_cause_r := BIT_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;

                        --------------------------------------------------------
                        -- ACK FIELD, ACK slot
                        --   * CAN_CONTROLLER_REQ_BAS1
                        --   * The ACK FIELD is two bits long and contains the ACK SLOT and the ACK DELIMITER (CAN_CONTROLLER_REQ_DF6)
                        --
                        when ACK_SLOT =>
                            -- pragma translate_off
                            assert s_rx_is_stuff_bit_r = '0' severity failure;
                            assert s_tx_stuff_bit_r = '0' severity failure;
                            -- pragma translate_on  

                            -- TX logic
                            -- --------
                            if s_transmitting_r = '1' and i_btl_rx_bit = '0' then
                                s_is_startup_r <= '0'; -- startup phase is over after the first ACK has been received (CAN_CONTROLLER_REQ_FC5)
                            end if;

                            -- Next state logic
                            -- ----------------
                            s_bit_idx_r   <= 0;
                            s_can_field_r <= ACK_DELIMITER;

                            -- Error handling
                            -- --------------
                            if s_transmitting_r = '1' and i_btl_rx_bit /= '0' and s_testmode_r = NORMAL_OPERATION then
                                -- ACK error (only in normal operation, do not report this in listen-only and loopback modes)
                                s_ack_error_r        <= '1'; -- CAN_CONTROLLER_REQ_ERR5
                                v_error_flag_cause_r := ACK_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;

                        --------------------------------------------------------
                        -- ACK FIELD, ACK delimiter
                        --   * The ACK FIELD is two bits long and contains the ACK SLOT and the ACK DELIMITER (CAN_CONTROLLER_REQ_DF6)
                        --
                        when ACK_DELIMITER =>
                            -- pragma translate_off
                            assert s_rx_is_stuff_bit_r = '0' severity failure;
                            assert s_tx_stuff_bit_r = '0' severity failure;
                            -- pragma translate_on

                            -- Next state logic
                            -- ----------------
                            s_bit_idx_r   <= 0;
                            s_can_field_r <= EOF;

                            -- Error handling
                            -- --------------                                                                                        
                            if s_receiving_r = '1' and i_btl_rx_bit /= '1' then -- ACK delimiter form error
                                s_form_error_r       <= '1'; -- a FORM ERROR has to be detected when a fixed-form bit field contains one or more illegal bits. (CAN_CONTROLLER_REQ_ERR2)
                                v_error_flag_cause_r := FORM_ERROR;
                                increase_rx_error_count(1); -- increase received error count by 1 (CAN_CONTROLLER_REQ_FC4a)
                                s_bit_idx_r   <= 0;
                                s_can_field_r <= ERROR_FLAG;
                            elsif s_receiving_r = '1' and s_rx_msg_received_crc_r /= s_rx_msg_computed_crc_r then -- A CRC ERROR has to be detected, if the calculated result is not the same as that received in the CRC sequence. (CAN_CONTROLLER_REQ_ERR3)
                                -- Whenever a CRC ERROR is detected, transmission 
                                -- of an ERROR FLAG starts at the bit following 
                                -- the ACK DELIMITER, unless an ERROR FLAG for 
                                -- another condition has already been started. (CAN_CONTROLLER_REQ_ERR6d)                                
                                s_crc_error_r        <= '1';
                                v_error_flag_cause_r := CRC_ERROR;
                                increase_rx_error_count(1); -- increase received error count by 1 (CAN_CONTROLLER_REQ_FC4a)
                                s_bit_idx_r   <= 0;
                                s_can_field_r <= ERROR_FLAG;
                            end if;
                            if s_transmitting_r = '1' and s_tx_rx_bit_mismatch_r = '1' then
                                -- BIT ERROR while transmitting
                                s_bit_error_r        <= '1'; -- BIT ERROR (CAN_CONTROLLER_REQ_ERR4)
                                v_error_flag_cause_r := BIT_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;

                        --------------------------------------------------------
                        -- End of frame
                        --
                        when EOF =>
                            -- pragma translate_off
                            assert s_rx_is_stuff_bit_r = '0' severity failure;
                            assert s_tx_stuff_bit_r = '0' severity failure;
                            -- pragma translate_on

                            -- RX logic
                            -- --------                            
                            if s_receiving_r = '1' and s_bit_idx_r = EOF_LENGTH_BITS - 1 then -- CAN_CONTROLLER_REQ_DF10
                                -- Message received successfully 
                                if s_transmitting_r = '0' then -- could be asserted at the same time as s_receiving_r in loopback mode
                                    decrease_rx_error_count(1); -- CAN_CONTROLLER_REQ_FC4k
                                end if;
                                s_rx_msg_valid_r <= '1'; -- The message is valid for the receivers, if there is no error until the last but one bit of END OF FRAME. (CAN_CONTROLLER_REQ_VAL1)
                            end if;

                            -- Next state logic
                            -- ----------------                            
                            if s_bit_idx_r = EOF_LENGTH_BITS - 1 then -- CAN_CONTROLLER_REQ_DF10
                                s_bit_idx_r   <= 0;
                                s_can_field_r <= INTERFRAME_INTERMISSION;
                            else
                                s_bit_idx_r <= s_bit_idx_r + 1;
                            end if;

                            -- Error handling
                            -- --------------
                            if s_receiving_r = '1' and i_btl_rx_bit /= '1' then
                                if s_bit_idx_r < EOF_LENGTH_BITS - 1 then
                                    -- A FORM ERROR has to be detected when a fixed-form bit field contains one or more illegal bits. (CAN_CONTROLLER_REQ_ERR2)
                                    s_form_error_r       <= '1';
                                    v_error_flag_cause_r := FORM_ERROR;
                                    increase_rx_error_count(1); -- increase received error count by 1 (CAN_CONTROLLER_REQ_FC4a)
                                    s_rx_msg_valid_r <= '0';
                                    s_bit_idx_r      <= 0;
                                    s_can_field_r    <= ERROR_FLAG;
                                else
                                    -- For a Receiver a dominant bit during the last bit of END OF FRAME is not treated as FORM ERROR (CAN_CONTROLLER_REQ_ERR2a)
                                    -- Respond to a dominant bit during the last bit of END OF FRAME with an OVERLOAD FRAME (CAN_CONTROLLER_REQ_ADD4)
                                    s_bit_idx_r   <= 0;
                                    s_can_field_r <= OVERLOAD_FLAG;
                                end if;
                            end if;
                            if s_transmitting_r = '1' and s_tx_rx_bit_mismatch_r = '1' then
                                -- BIT ERROR while transmitting
                                s_bit_error_r        <= '1'; -- BIT ERROR (CAN_CONTROLLER_REQ_ERR4)
                                v_error_flag_cause_r := BIT_ERROR;
                                s_bit_idx_r          <= 0;
                                s_can_field_r        <= ERROR_FLAG;
                            end if;

                        --------------------------------------------------------
                        -- ACTIVE ERROR FLAG
                        --   * A station detecting an error condition signals this by transmitting an ERROR FLAG (CAN_CONTROLLER_REQ_ERR6)
                        --   * Whenever a BIT ERROR, a STUFF ERROR, a FORM ERROR or an ACKNOWLEDGMENT ERROR is detected by any station, 
                        --     transmission of an ERROR FLAG is started at the respective station at the next bit. (CAN_CONTROLLER_REQ_ERR6c)
                        --   * OVERLOAD FRAMEs and ERROR FRAMEs are not preceded by an INTERFRAME SPACE (CAN_CONTROLLER_REQ_IF1a)
                        --
                        when ACTIVE_ERROR_FLAG =>
                            -- pragma translate_off
                            assert s_rx_is_stuff_bit_r = '0' severity failure;
                            assert s_tx_stuff_bit_r = '0' severity failure;
                            assert v_error_flag_cause_r /= NONE report "error flag cause must be reset before entering ACTIVE_ERROR_FLAG state" severity failure;
                            -- pragma translate_on

                            -- TX logic
                            -- --------
                            if s_transmitting_r = '1' and not v_arbitration_tx_stuff_error_r and s_bit_idx_r = ERROR_FLAG_LENGTH_BITS - 1 then -- CAN_CONTROLLER_REQ_FC4e
                                increase_tx_error_count(8); -- When a TRANSMITTER sends an ERROR FLAG the TRANSMIT ERROR COUNT is increased by 8. (CAN_CONTROLLER_REQ_FC4c)
                            end if;

                            -- Next state logic
                            -- ----------------
                            if s_bit_idx_r = ERROR_FLAG_LENGTH_BITS - 1 then -- last bit of ACTIVE ERROR FLAG                                
                                s_bit_idx_r   <= 0;
                                s_can_field_r <= ERROR_FLAG_ECHO;
                            else
                                s_bit_idx_r <= s_bit_idx_r + 1;
                            end if;

                            -- Error handling
                            -- --------------                            
                            if i_btl_rx_bit /= '0' then
                                if s_transmitting_r = '1' then
                                    s_bit_error_r <= '1';
                                    increase_tx_error_count(8); -- If a TRANSMITTER detects a BIT ERROR while sending an ACTIVE ERROR FLAG or an OVERLOAD FLAG the TRANSMIT ERROR COUNT is increased by 8. (CAN_CONTROLLER_REQ_FC4g)
                                end if;
                                if s_receiving_r = '1' then
                                    s_bit_error_r <= '1';
                                    increase_rx_error_count(8); -- If an RECEIVER detects a BIT ERROR while sending an ACTIVE ERROR FLAG or an OVERLOAD FLAG the RECEIVE ERROR COUNT is increased by 8. (CAN_CONTROLLER_REQ_FC4h)
                                end if;
                            end if;

                        --------------------------------------------------------
                        -- PASSIVE ERROR FLAG
                        --   * A TRANSMITTER sending a PASSIVE ERROR FLAG and detecting a 'dominant' bit does not interpret this as a BIT ERROR. (CAN_CONTROLLER_REQ_ERR4b)
                        --
                        when PASSIVE_ERROR_FLAG =>
                            -- pragma translate_off
                            assert s_rx_is_stuff_bit_r = '0' severity failure;
                            assert s_tx_stuff_bit_r = '0' severity failure;
                            assert v_error_flag_cause_r /= NONE report "error flag cause must be reset before entering PASSIVE_ERROR_FLAG state" severity failure;
                            if s_can_field_old_r /= PASSIVE_ERROR_FLAG then
                                assert v_temp_bit_count_r = 0 report "bit count must be reset before entering PASSIVE_ERROR_FLAG state" severity failure;
                                assert v_dominant_bit_detected_r = false report "dominant bit detected must be reset before entering PASSIVE_ERROR_FLAG state" severity failure;
                            end if;
                            -- pragma translate_on

                            -- RX logic
                            -- --------                            
                            if i_btl_rx_bit = '0' then
                                v_dominant_bit_detected_r := true;
                            end if;

                            -- Next state logic
                            -- ----------------
                            -- The 'error passive' station waits for six consecutive bits of equal polarity, 
                            -- beginning at the start of the PASSIVE ERROR FLAG. The PASSIVE ERROR FLAG is 
                            -- complete when these 6 equal bits have been detected. (CAN_CONTROLLER_REQ_EF2a)
                            if v_temp_bit_count_r = 0 then
                                v_temp_bit_count_r          := 1;
                                v_error_flag_bit_polarity_r := i_btl_rx_bit;
                            elsif i_btl_rx_bit = v_error_flag_bit_polarity_r then
                                if v_temp_bit_count_r = ERROR_FLAG_LENGTH_BITS - 1 then -- last bit of PASSIVE ERROR FLAG
                                    if s_transmitting_r = '1' then -- When a TRANSMITTER sends an ERROR FLAG the TRANSMIT ERROR COUNT is increased by 8. (CAN_CONTROLLER_REQ_FC4c)
                                        if v_error_flag_cause_r = ACK_ERROR and not v_dominant_bit_detected_r then
                                            -- Exception 1: do not increment transmit error count in case of 'error passive' transmitter
                                            -- that detects an ACK error and does not detect a dominant bit while sending PASSIVE ERROR FLAG (CAN_CONTROLLER_REQ_FC4d, CAN_CONTROLLER_REQ_FC4f)
                                            v_tx_error_count_u9 := v_tx_error_count_u9;
                                        elsif v_arbitration_tx_stuff_error_r then
                                            -- Exception 2 (CAN_CONTROLLER_REQ_FC4e)
                                            v_tx_error_count_u9 := v_tx_error_count_u9;
                                        elsif s_is_startup_r = '1' and v_error_flag_cause_r = ACK_ERROR and v_tx_error_count_u9 + 8 >= BUS_OFF_LEVEL then
                                            -- Node should not go BUS OFF because of ACK error during startup (CAN_CONTROLLER_REQ_FC5)
                                            v_tx_error_count_u9 := v_tx_error_count_u9;
                                        else
                                            increase_tx_error_count(8);
                                        end if;
                                    end if;
                                    --
                                    v_temp_bit_count_r        := 0;
                                    v_dominant_bit_detected_r := false;
                                    s_can_field_r             <= ERROR_FLAG_ECHO;
                                else
                                    v_temp_bit_count_r := v_temp_bit_count_r + 1;
                                end if;
                            else
                                v_temp_bit_count_r := 0;
                            end if;
                            --
                            s_bit_idx_r <= v_temp_bit_count_r;

                        --------------------------------------------------------
                        when ERROR_FLAG_ECHO =>
                            -- pragma translate_off
                            assert s_rx_is_stuff_bit_r = '0' severity failure;
                            assert s_tx_stuff_bit_r = '0' severity failure;
                            -- pragma translate_on

                            -- RX logic
                            -- --------                            
                            -- When a RECEIVER detects a 'dominant' bit as the first bit after sending an ERROR FLAG the 
                            -- RECEIVE ERROR COUNT will be increased by 8. (CAN_CONTROLLER_REQ_FC4b)
                            if s_receiving_r = '1' and s_transmitting_r = '0' then
                                if i_btl_rx_bit = '0' and s_can_field_old_r /= ERROR_FLAG_ECHO then
                                    increase_rx_error_count(8);
                                end if;
                            end if;

                            -- Next state logic
                            -- ----------------                           
                            -- Count number of consecutive dominant bits. After detecting 8 of them, increase
                            -- the appropriate error count by 8 (CAN_CONTROLLER_REQ_FC4i)
                            if i_btl_rx_bit = '0' then
                                if s_bit_idx_r = 7 then
                                    s_bit_idx_r <= 0;
                                    if s_receiving_r = '1' then
                                        increase_rx_error_count(8);
                                    else
                                        increase_tx_error_count(8);
                                    end if;
                                else
                                    s_bit_idx_r <= s_bit_idx_r + 1;
                                end if;
                            else
                                v_temp_bit_count_r := 0;
                            end if;
                            --
                            if i_btl_rx_bit = '1' then -- Keep sending recessive bits until receiving the first recessive bit (CAN_CONTROLLER_REQ_EF3a)
                                s_bit_idx_r   <= 0;
                                s_can_field_r <= ERROR_DELIMITER;
                            end if;

                        --------------------------------------------------------
                        when ERROR_DELIMITER =>
                            -- pragma translate_off
                            assert s_rx_is_stuff_bit_r = '0' severity failure;
                            assert s_tx_stuff_bit_r = '0' severity failure;
                            -- pragma translate_on

                            -- Next state logic
                            -- ----------------                            
                            -- Transmit seven more recessive bits (CAN_CONTROLLER_REQ_EF3b)
                            if s_bit_idx_r = ERROR_DELIMITER_LENGTH_BITS - 2 then
                                if i_btl_rx_bit = '0' then -- If a CAN node samples a dominant bit at the eighth bit (the last bit) of an ERROR DELIMITER or OVERLOAD DELIMITER, it will start transmitting an OVERLOAD FRAME (not an ERROR FRAME). (CAN_CONTROLLER_REQ_OV4)
                                    s_bit_idx_r   <= 0;
                                    s_can_field_r <= OVERLOAD_FLAG; -- CAN_CONTROLLER_REQ_OV5
                                else
                                    s_bit_idx_r   <= 0;
                                    s_can_field_r <= INTERFRAME_INTERMISSION;
                                end if;
                            else
                                s_bit_idx_r <= s_bit_idx_r + 1;
                            end if;

                        --------------------------------------------------------
                        -- OVERLOAD FLAG
                        --   * OVERLOAD FRAMEs and ERROR FRAMEs are not preceded by an INTERFRAME SPACE (CAN_CONTROLLER_REQ_IF1a)
                        --   * The Error Counters will not be incremented (CAN_CONTROLLER_REQ_OV4a)
                        --                   
                        when OVERLOAD_FLAG =>
                            -- pragma translate_off
                            assert s_rx_is_stuff_bit_r = '0' severity failure;
                            assert s_tx_stuff_bit_r = '0' severity failure;
                            -- pragma translate_on

                            -- Next state logic
                            -- ----------------                            
                            if s_bit_idx_r = OVERLOAD_FLAG_LENGTH_BITS - 1 then
                                s_overload_r  <= '1';
                                s_bit_idx_r   <= 0;
                                s_can_field_r <= OVERLOAD_FLAG_ECHO;
                            else
                                s_bit_idx_r <= s_bit_idx_r + 1;
                            end if;

                            -- Error handling
                            -- --------------
                            if i_btl_rx_bit /= '0' then
                                if s_transmitting_r = '1' then
                                    s_bit_error_r <= '1';
                                    increase_tx_error_count(8); -- If a TRANSMITTER detects a BIT ERROR while sending an ACTIVE ERROR FLAG or an OVERLOAD FLAG the TRANSMIT ERROR COUNT is increased by 8. (CAN_CONTROLLER_REQ_FC4g)
                                end if;
                                if s_receiving_r = '1' then
                                    s_bit_error_r <= '1';
                                    increase_rx_error_count(8); -- If an RECEIVER detects a BIT ERROR while sending an ACTIVE ERROR FLAG or an OVERLOAD FLAG the RECEIVE ERROR COUNT is increased by 8. (CAN_CONTROLLER_REQ_FC4h)
                                end if;
                            end if;

                        --------------------------------------------------------
                        when OVERLOAD_FLAG_ECHO =>
                            -- pragma translate_off
                            assert s_rx_is_stuff_bit_r = '0' severity failure;
                            assert s_tx_stuff_bit_r = '0' severity failure;
                            -- pragma translate_on       

                            -- Next state logic
                            -- ----------------                            
                            if i_btl_rx_bit = '1' then -- Keep sending recessive bits until receiving the first recessive bit (CAN_CONTROLLER_REQ_OV6)
                                s_bit_idx_r   <= 0;
                                s_can_field_r <= OVERLOAD_DELIMITER;
                            end if;

                        --------------------------------------------------------
                        when OVERLOAD_DELIMITER =>
                            -- pragma translate_off
                            assert s_rx_is_stuff_bit_r = '0' severity failure;
                            assert s_tx_stuff_bit_r = '0' severity failure;
                            -- pragma translate_on

                            -- Next state logic
                            -- ----------------                            
                            -- Transmit seven more recessive bits (CAN_CONTROLLER_REQ_OV6a)
                            if s_bit_idx_r = OVERLOAD_DELIMITER_LENGTH_BITS - 2 then
                                if i_btl_rx_bit = '0' then -- If a CAN node samples a dominant bit at the eighth bit (the last bit) of an ERROR DELIMITER or OVERLOAD DELIMITER, it will start transmitting an OVERLOAD FRAME (not an ERROR FRAME). (CAN_CONTROLLER_REQ_OV4)
                                    s_bit_idx_r   <= 0;
                                    s_can_field_r <= OVERLOAD_FLAG; -- multiple OVERLOAD FRAMEs are not separated by an INTERFRAME SPACE. (CAN_CONTROLLER_REQ_IF1b, CAN_CONTROLLER_REQ_OV5)
                                else
                                    s_bit_idx_r   <= 0;
                                    s_can_field_r <= INTERFRAME_INTERMISSION;
                                end if;
                            else
                                s_bit_idx_r <= s_bit_idx_r + 1;
                            end if;

                        --------------------------------------------------------
                        -- INTERFRAME SPACE, intermission
                        --
                        --   * 3 recessive bits (CAN_CONTROLLER_REQ_IF3)
                        --
                        --   * DATA FRAMEs and REMOTE FRAMEs are separated from preceding frames whatever type they are (DATA FRAME, 
                        --     REMOTE FRAME, ERROR FRAME OVERLOAD FRAME) by a bit field called INTERFRAME SPACE. (CAN_CONTROLLER_REQ_IF1)
                        --
                        --   * During INTERMISSION the only action to be taken is signaling an OVERLOAD condition and no station is 
                        --     allowed to actively start transmission of a DATA FRAME or REMOTE FRAME. (CAN_CONTROLLER_REQ_IF3a)
                        --
                        when INTERFRAME_INTERMISSION =>
                            -- pragma translate_off
                            assert s_rx_is_stuff_bit_r = '0' severity failure;
                            assert s_tx_stuff_bit_r = '0' severity failure;
                            -- pragma translate_on

                            -- TX logic
                            -- --------                            
                            -- Successful message transmission:
                            if s_bit_idx_r = 0 and s_transmitting_r = '1' and s_can_field_old_r = EOF then -- do not assert "ready" when coming from an ERROR state
                                decrease_tx_error_count(1); -- CAN_CONTROLLER_REQ_FC4j
                                s_tx_msg_ready_r <= '1'; -- The message is valid for the transmitter, if there is no error until the end of END OF FRAME. (CAN_CONTROLLER_REQ_VAL2)
                            end if;

                            -- Next state logic
                            -- ----------------                            
                            if s_bit_idx_r = INTERMISSION_LENGTH_BITS - 1 then -- last (third) bit of INTERMISSION                                
                                if i_user_tx_msg_valid = '1' and -- message pending for transmission 
                                (i_btl_hard_sync = '1' or -- sampled a dominant bit -> interpret this as a SOF (CAN_CONTROLLER_REQ_IF3b)
                                    s_receiving_r = '1') -- during transmission of another message (CAN_CONTROLLER_REQ_IF4a) 
                                then
                                    s_receiving_r             <= '1';
                                    s_transmitting_r          <= '1'; -- A unit originating a message is called "TRANSMITTER" of that message. (CAN_CONTROLLER_REQ_DEF1)
                                    s_bit_idx_r               <= 0;
                                    v_dominant_bit_detected_r := false;
                                    v_error_flag_cause_r      := NONE;
                                    s_can_field_r             <= SOF;
                                elsif s_transmitting_r = '1' and s_error_state = ERROR_PASSIVE then
                                    -- ERROR PASSIVE transmitter -> SUSPEND TRANSMISSION (CAN_CONTROLLER_REQ_IF2, CAN_CONTROLLER_REQ_FC2a)
                                    s_bit_idx_r   <= 0;
                                    s_can_field_r <= SUSPEND_TRANSMISSION;
                                else
                                    -- ERROR ACTIVE receiver/transmitter or ERROR PASSIVE receiver -> BUS IDLE
                                    s_bit_idx_r   <= 0;
                                    s_can_field_r <= BUS_IDLE;
                                end if;
                            else
                                s_bit_idx_r <= s_bit_idx_r + 1;
                            end if;

                            -- Error handling
                            -- --------------                            
                            -- Detection of a dominant bit at the first and second bits of INTERMISSION leads to the transmission 
                            -- of an OVERLOAD flag (CAN_CONTROLLER_REQ_OV3)
                            if s_bit_idx_r <= 1 and i_btl_rx_bit = '0' then
                                s_bit_idx_r   <= 0;
                                s_can_field_r <= OVERLOAD_FLAG; -- CAN_CONTROLLER_REQ_OV5
                            end if;

                        --------------------------------------------------------
                        -- INTERFRAME SPACE, suspend transmission
                        --                            
                        when SUSPEND_TRANSMISSION =>
                            -- pragma translate_off
                            assert s_rx_is_stuff_bit_r = '0' severity failure;
                            assert s_tx_stuff_bit_r = '0' severity failure;
                            -- pragma translate_on

                            -- Next state logic
                            -- ----------------                        
                            if s_bit_idx_r = SUSPEND_TRANSMISSION_LENGTH_BITS - 1 then
                                s_bit_idx_r   <= 0;
                                s_can_field_r <= BUS_IDLE;
                            else
                                s_bit_idx_r <= s_bit_idx_r + 1;
                            end if;
                            -- If meanwhile a transmission (caused by another station) starts, the station will become receiver of this message. (CAN_CONTROLLER_REQ_IF5a)
                            if i_btl_hard_sync = '1' then
                                s_receiving_r             <= '1'; -- A unit is called "RECEIVER" of a message, if it is not TRANSMITTER of that message and the bus is not idle. (CAN_CONTROLLER_REQ_DEF2)
                                s_transmitting_r          <= '0'; -- The unit stays TRANSMITTER until the bus is idle or the unit loses ARBITRATION. (CAN_CONTROLLER_REQ_DEF1)
                                s_bit_idx_r               <= 0;
                                v_dominant_bit_detected_r := false;
                                v_error_flag_cause_r      := NONE;
                                s_can_field_r             <= SOF;
                            end if;
                    end case;

                    -- Check for BUS OFF condition:
                    if s_error_state = BUS_OFF then
                        s_can_field_r <= SYNCHRONIZE; -- in case of BUS OFF, remain in SYNCHRONIZE state until reset (CAN_CONTROLLER_REQ_FC4p)
                    end if;
                    --
                    s_can_field_old_r     <= s_can_field_r;
                    s_rx_error_count_u8_r <= v_rx_error_count_u8;
                    s_tx_error_count_u9_r <= v_tx_error_count_u9;
                    s_tx_stuff_value_r    <= not s_btl_tx_bit;

                end if;                 -- i_btl_tx_clk = '1'
            end if;                     -- i_reset = '1'
        end if;                         -- rising_edge(i_clk)
    end process bsp_fsm_seq;

    bsp_fsm_comb : process(s_can_field_r, s_transmitting_r, s_tx_msg_id_shreg_r, i_user_tx_msg, s_tx_msg_data_shreg_r, s_tx_msg_dlc_shreg_r, s_receiving_r, s_rx_msg_computed_crc_r, s_rx_msg_received_crc_r, s_is_stuff_bit_r, s_tx_stuff_value_r, s_tx_msg_crc_shreg_r) is
    begin
        -- defaults:
        s_btl_tx_bit <= '1';
        s_stuff_en   <= '1';

        case s_can_field_r is
            when SYNCHRONIZE =>
                s_stuff_en <= '0';
                s_crc_en   <= '0';

            when BUS_IDLE =>
                s_stuff_en <= '0';
                s_crc_en   <= '0';

            when SOF =>
                s_crc_en <= '1';        -- CAN_CONTROLLER_REQ_DF11
                if s_transmitting_r = '1' then
                    s_btl_tx_bit <= '0';
                end if;

            when ID11 =>
                s_crc_en <= '1';        -- CAN_CONTROLLER_REQ_DF11
                if s_transmitting_r = '1' then
                    s_btl_tx_bit <= s_tx_msg_id_shreg_r(s_tx_msg_id_shreg_r'high);
                end if;

            when RTR_OR_SRR =>
                s_crc_en <= '1';        -- CAN_CONTROLLER_REQ_DF11
                if s_transmitting_r = '1' then
                    if i_user_tx_msg.ide = '0' then -- standard format
                        s_btl_tx_bit <= i_user_tx_msg.rtr; -- In DATA FRAMEs the RTR BIT has to be 'dominant'. Within a REMOTE FRAME the RTR BIT has to be 'recessive'. (CAN_CONTROLLER_REQ_DF14)
                    else                -- extended format
                        s_btl_tx_bit <= '1'; -- The SRR is a recessive bit. (CAN_CONTROLLER_REQ_DF12)
                    end if;
                end if;

            when IDE =>
                s_crc_en <= '1';        -- CAN_CONTROLLER_REQ_DF11
                if s_transmitting_r = '1' then
                    s_btl_tx_bit <= i_user_tx_msg.ide; -- The IDE bit in the Standard Format is transmitted 'dominant', whereas in the Extended Format the IDE bit is recessive (CAN_CONTROLLER_REQ_DF13)
                end if;

            when ID18 =>
                s_crc_en <= '1';        -- CAN_CONTROLLER_REQ_DF11
                if s_transmitting_r = '1' then
                    s_btl_tx_bit <= s_tx_msg_id_shreg_r(s_tx_msg_id_shreg_r'high);
                end if;

            when RTR_EXT =>
                s_crc_en <= '1';        -- CAN_CONTROLLER_REQ_DF11
                if s_transmitting_r = '1' then
                    s_btl_tx_bit <= i_user_tx_msg.rtr; -- In DATA FRAMEs the RTR BIT has to be 'dominant'. Within a REMOTE FRAME the RTR BIT has to be 'recessive'. (CAN_CONTROLLER_REQ_DF14)
                end if;

            when R1 =>
                s_crc_en <= '1';        -- CAN_CONTROLLER_REQ_DF11
                if s_transmitting_r = '1' then
                    s_btl_tx_bit <= '0'; -- reserved bits have to be sent dominant (CAN_CONTROLLER_REQ_DF15)
                end if;

            when R0 =>
                s_crc_en <= '1';        -- CAN_CONTROLLER_REQ_DF11
                if s_transmitting_r = '1' then
                    s_btl_tx_bit <= '0'; -- reserved bits have to be sent dominant (CAN_CONTROLLER_REQ_DF15)
                end if;

            when DLC =>
                s_crc_en <= '1';        -- CAN_CONTROLLER_REQ_DF11
                if s_transmitting_r = '1' then
                    s_btl_tx_bit <= s_tx_msg_dlc_shreg_r(s_tx_msg_dlc_shreg_r'high);
                end if;

            when DATA =>
                s_crc_en <= '1';        -- CAN_CONTROLLER_REQ_DF11
                if s_transmitting_r = '1' then
                    s_btl_tx_bit <= s_tx_msg_data_shreg_r(s_tx_msg_data_shreg_r'high);
                end if;

            when CRC_SEQ =>
                s_crc_en <= '0';
                if s_transmitting_r = '1' then
                    s_btl_tx_bit <= s_tx_msg_crc_shreg_r(s_tx_msg_crc_shreg_r'high);
                end if;

            when CRC_DELIMITER =>
                s_crc_en   <= '0';
                s_stuff_en <= '0';      -- CAN_CONTROLLER_REQ_COD1b
                if s_transmitting_r = '1' then
                    s_btl_tx_bit <= '1'; -- CRC DELIMITER consists of a single 'recessive' bit (CAN_CONTROLLER_REQ_DF5)
                end if;

            when ACK_SLOT =>
                s_crc_en   <= '0';
                s_stuff_en <= '0';      -- CAN_CONTROLLER_REQ_COD1b
                if s_transmitting_r = '1' then
                    s_btl_tx_bit <= '1'; -- In the ACK FIELD the transmitting station sends two 'recessive' bits (CAN_CONTROLLER_REQ_DF7)
                end if;
                --
                if s_receiving_r = '1' then
                    if s_rx_msg_received_crc_r /= s_rx_msg_computed_crc_r then
                        s_btl_tx_bit <= '1'; -- do not acknowledge in case of CRC error (CAN_CONTROLLER_REQ_DF24)
                    else
                        s_btl_tx_bit <= '0'; -- acknowledge the received message (CAN_CONTROLLER_REQ_DF8)
                    end if;
                end if;

            when ACK_DELIMITER =>
                s_crc_en   <= '0';
                s_stuff_en <= '0';      -- CAN_CONTROLLER_REQ_COD1b
                if s_transmitting_r = '1' then
                    s_btl_tx_bit <= '1'; -- In the ACK FIELD the transmitting station sends two 'recessive' bits (CAN_CONTROLLER_REQ_DF7, CAN_CONTROLLER_REQ_DF9)
                end if;

            when EOF =>
                s_crc_en   <= '0';
                s_stuff_en <= '0';      -- CAN_CONTROLLER_REQ_COD1b
                if s_transmitting_r = '1' then
                    s_btl_tx_bit <= '1'; -- Each DATA FRAME and REMOTE FRAME is delimited by a flag sequence consisting of seven 'recessive' bits. (CAN_CONTROLLER_REQ_DF10)
                end if;

            when ACTIVE_ERROR_FLAG =>
                s_crc_en     <= '0';
                s_stuff_en   <= '0';    -- CAN_CONTROLLER_REQ_COD1b
                s_btl_tx_bit <= '0';    -- active error flag (CAN_CONTROLLER_REQ_ERR6a, CAN_CONTROLLER_REQ_EF1, CAN_CONTROLLER_REQ_FC1)

            when PASSIVE_ERROR_FLAG =>
                s_crc_en     <= '0';
                s_stuff_en   <= '0';    -- CAN_CONTROLLER_REQ_COD1b
                s_btl_tx_bit <= '1';    -- passive error flag (CAN_CONTROLLER_REQ_ERR6b, CAN_CONTROLLER_REQ_EF2, CAN_CONTROLLER_REQ_FC2)

            when ERROR_FLAG_ECHO =>
                s_crc_en     <= '0';
                s_stuff_en   <= '0';    -- CAN_CONTROLLER_REQ_COD1c
                s_btl_tx_bit <= '1';

            when ERROR_DELIMITER =>
                s_crc_en     <= '0';
                s_stuff_en   <= '0';    -- CAN_CONTROLLER_REQ_COD1c
                s_btl_tx_bit <= '1';    -- eight recessive bits (CAN_CONTROLLER_REQ_EF3)

            when OVERLOAD_FLAG =>
                s_crc_en     <= '0';
                s_stuff_en   <= '0';    -- CAN_CONTROLLER_REQ_COD1c
                s_btl_tx_bit <= '0';    -- six dominant bits (CAN_CONTROLLER_REQ_OV1)

            when OVERLOAD_FLAG_ECHO =>
                s_crc_en     <= '0';
                s_stuff_en   <= '0';    -- CAN_CONTROLLER_REQ_COD1c
                s_btl_tx_bit <= '1';

            when OVERLOAD_DELIMITER =>
                s_crc_en     <= '0';
                s_stuff_en   <= '0';    -- CAN_CONTROLLER_REQ_COD1c
                s_btl_tx_bit <= '1';    -- eight recessive bits (CAN_CONTROLLER_REQ_OV2)

            when INTERFRAME_INTERMISSION =>
                s_crc_en   <= '0';
                s_stuff_en <= '0';      -- CAN_CONTROLLER_REQ_COD1b
                if s_transmitting_r = '1' then
                    s_btl_tx_bit <= '1';
                end if;

            when SUSPEND_TRANSMISSION =>
                s_crc_en   <= '0';
                s_stuff_en <= '0';      -- CAN_CONTROLLER_REQ_COD1b
                -- pragma translate_off
                assert s_transmitting_r = '1' severity failure;
                -- pragma translate_on
                s_btl_tx_bit <= '1';    -- eight recessive bits (CAN_CONTROLLER_REQ_IF5)

        end case;

        -- Transmit a stuffing bit:
        if s_transmitting_r = '1' and s_is_stuff_bit_r = '1' then
            s_btl_tx_bit <= s_tx_stuff_value_r;
        end if;

    end process bsp_fsm_comb;

    -------------------------------------------------------------------------------
    -- DESTUFFER
    --
    destuffer : process(i_clk) is
        variable v_identical_rx_bit_count_r : natural range 0 to MAX_IDENTICAL_BITS;
        variable v_identical_rx_bit_value_r : std_logic := '1';
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' or s_stuff_en = '0' then
                v_identical_rx_bit_count_r := 0;
                v_identical_rx_bit_value_r := '1';
                s_rx_is_stuff_bit_r        <= '0';
                s_rx_stuff_error_r         <= '0';
                s_tx_stuff_bit_r           <= '0';
                s_is_stuff_bit_r           <= '0';
            elsif i_btl_rx_clk = '1' then
                -- defaults:
                s_rx_stuff_error_r <= '0';
                s_tx_stuff_bit_r   <= '0';

                if v_identical_rx_bit_count_r = MAX_IDENTICAL_BITS then
                    if i_btl_rx_bit = v_identical_rx_bit_value_r then
                        s_rx_stuff_error_r <= '1'; -- CAN_CONTROLLER_REQ_ERR1
                    end if;
                    v_identical_rx_bit_count_r := 1;
                    v_identical_rx_bit_value_r := i_btl_rx_bit;
                    s_rx_is_stuff_bit_r        <= '1';
                else
                    if i_btl_rx_bit = v_identical_rx_bit_value_r then
                        v_identical_rx_bit_count_r := v_identical_rx_bit_count_r + 1;
                        s_rx_is_stuff_bit_r        <= '0';
                        if v_identical_rx_bit_count_r = MAX_IDENTICAL_BITS then
                            s_tx_stuff_bit_r <= '1';
                        end if;
                    else
                        v_identical_rx_bit_count_r := 1;
                        v_identical_rx_bit_value_r := i_btl_rx_bit;
                        s_rx_is_stuff_bit_r        <= '0';
                    end if;
                end if;
            elsif i_btl_tx_clk = '1' then
                s_is_stuff_bit_r <= s_tx_stuff_bit_r;
            end if;
        end if;
    end process destuffer;

    -------------------------------------------------------------------------------
    -- BIT ERROR DETECTOR
    --
    --   * Compare the transmitted and received bit values and signal mismatches 
    --
    bit_error_detector : process(i_clk) is
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                s_tx_rx_bit_mismatch_r <= '0';
            elsif i_btl_rx_clk = '1' then
                s_tx_rx_bit_mismatch_r <= '0';
                if i_btl_rx_bit /= s_btl_tx_bit then
                    s_tx_rx_bit_mismatch_r <= '1';
                end if;
            end if;
        end if;
    end process bit_error_detector;

    -------------------------------------------------------------------------------
    -- CRC CALCULATION
    --
    crc : process(i_clk) is
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' or s_crc_en = '0' then
                s_crc_r <= CRC_INIT;
            elsif i_btl_rx_clk = '1' and s_is_stuff_bit_r = '0' then
                s_crc_r <= update_crc(i_btl_rx_bit, s_crc_r);
            end if;
        end if;
    end process crc;

    -------------------------------------------------------------------------------
    -- ERROR STATE 
    --  
    error_state : process(s_rx_error_count_u8_r, s_tx_error_count_u9_r) is
    begin
        if s_tx_error_count_u9_r >= BUS_OFF_LEVEL then -- CAN_CONTROLLER_REQ_FC4n        
            s_error_state <= BUS_OFF;
        elsif s_tx_error_count_u9_r >= ERROR_PASSIVE_LEVEL or s_rx_error_count_u8_r >= ERROR_PASSIVE_LEVEL then -- CAN_CONTROLLER_REQ_FC4l
            s_error_state <= ERROR_PASSIVE;
        else                            -- CAN_CONTROLLER_REQ_FC4o
            s_error_state <= ERROR_ACTIVE;
        end if;
    end process error_state;

    -------------------------------------------------------------------------------
    -- OUTPUTS 
    --  
    o_bus_idle <= '1' when s_can_field_r = BUS_IDLE or -- hard synchronization is allowed when bus is IDLE and
        s_can_field_r = SUSPEND_TRANSMISSION or -- during SUSPEND TRANSMISSION and
        (s_can_field_r = INTERFRAME_INTERMISSION and s_bit_idx_r = INTERMISSION_LENGTH_BITS - 1) -- during the last bit of INTERMISSION (CAN_CONTROLLER_REQ_ADD2)
        else '0';
    --
    o_tx_bit              <= s_btl_tx_bit;
    o_debug_can_field     <= s_can_field_r;
    o_debug_can_field_slv <= std_logic_vector(to_unsigned(t_can_field'pos(s_can_field_r), 5));
    o_debug_receiving     <= s_receiving_r;
    o_debug_transmitting  <= s_transmitting_r;
    o_debug_stuff_bit     <= s_is_stuff_bit_r;
    o_debug_bit_idx_u5    <= std_logic_vector(to_unsigned(s_bit_idx_r, 5));
    o_debug_byte_idx_u3   <= std_logic_vector(to_unsigned(s_byte_idx_r, 3));
    o_user_rx_msg         <= s_rx_msg_r;
    o_user_rx_msg_valid   <= s_rx_msg_valid_r;
    o_user_tx_msg_ready   <= s_tx_msg_ready_r;
    o_stuff_error         <= s_stuff_error_r;
    o_form_error          <= s_form_error_r;
    o_ack_error           <= s_ack_error_r;
    o_bit_error           <= s_bit_error_r;
    o_arb_loss            <= s_arb_loss_r;
    o_crc_error           <= s_crc_error_r;
    o_overload            <= s_overload_r;
    o_rx_error_count_u8   <= s_rx_error_count_u8_r;
    o_tx_error_count_u9   <= s_tx_error_count_u9_r;
    o_error_state         <= s_error_state;

end architecture RTL;
