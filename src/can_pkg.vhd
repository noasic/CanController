-------------------------------------------------------------------------------
--
-- CAN Package
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

package can_pkg is

    -- CAN message structure
    type t_can_msg is record
        id   : unsigned(28 downto 0);   -- Message identifier (use bits [10:0] for standard-length identifiers) 
        rtr  : std_logic;               -- Remote transmission request flag
        ide  : std_logic;               -- Extended identifier flag
        dlc  : unsigned(3 downto 0);    -- Data length code
        data : std_logic_vector(63 downto 0); -- Message data [7:0] byte #0, [63:56] byte #7
    end record t_can_msg;

    type t_can_field is (               -- index:
        SYNCHRONIZE,                    -- 0
        BUS_IDLE,                       -- 1
        SOF,                            -- 2
        ID11,                           -- 3
        RTR_OR_SRR,                     -- 4 
        IDE,                            -- 5
        R0,                             -- 6
        R1,                             -- 7
        ID18,                           -- 8
        RTR_EXT,                        -- 9
        DLC,                            -- 10
        DATA,                           -- 11
        CRC_SEQ,                        -- 12
        CRC_DELIMITER,                  -- 13
        ACK_SLOT,                       -- 14
        ACK_DELIMITER,                  -- 15
        EOF,                            -- 16
        INTERFRAME_INTERMISSION,        -- 17
        ACTIVE_ERROR_FLAG,              -- 18
        ERROR_FLAG_ECHO,                -- 19
        ERROR_DELIMITER,                -- 20
        OVERLOAD_FLAG,                  -- 21
        OVERLOAD_FLAG_ECHO,             -- 22
        OVERLOAD_DELIMITER,             -- 23
        SUSPEND_TRANSMISSION,           -- 24
        PASSIVE_ERROR_FLAG);            -- 25

    -- Compare two CAN messages 
    function "="(l : t_can_msg; r : t_can_msg) return boolean;

end package can_pkg;

package body can_pkg is
    function "="(l : t_can_msg; r : t_can_msg) return boolean is
    begin
        -- IDE
        if l.ide /= r.ide then
            return false;
        end if;
        -- ID
        if l.ide = '0' then             -- standard format
            if l.id(28 downto 18) /= r.id(28 downto 18) then
                return false;
            end if;
        else                            -- extended format
            if l.id /= r.id then
                return false;
            end if;
        end if;
        -- RTR
        if l.rtr /= r.rtr then
            return false;
        end if;
        -- DLC
        if l.dlc /= r.dlc then
            return false;
        end if;
        -- DATA
        if l.rtr = '1' or l.dlc = 0 then -- no data
            return true;
        end if;
        --
        for byte_idx in 0 to to_integer(l.dlc) - 1 loop
            if l.data(63 - 8 * byte_idx downto 56 - 8 * byte_idx) /= r.data(63 - 8 * byte_idx downto 56 - 8 * byte_idx) then
                return false;
            end if;
        end loop;
        --
        return true;
    end function;

end package body can_pkg;
