#!/usr/bin/env python3
"""
Enhanced Franka server with detailed logging for debugging.
Run this instead of franka_server.py to get detailed logs.
"""

import os
import sys
import logging
from datetime import datetime

# Setup logging before any imports
log_filename = f"franka_server_debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S',
    handlers=[
        logging.FileHandler(log_filename),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

logger.info("=" * 60)
logger.info("FRANKA SERVER DEBUG VERSION STARTING")
logger.info(f"Log file: {log_filename}")
logger.info("=" * 60)

# Add project root to Python path
project_root = os.path.dirname(os.path.abspath(__file__))
if project_root not in sys.path:
    sys.path.insert(0, project_root)
    logger.info(f"Added {project_root} to Python path")

try:
    from frankateach.franka_server import FrankaServer
    from frankateach.utils import notify_component_start
    import hydra
    import time
    import numpy as np
    logger.info("✅ Successfully imported all modules")
except Exception as e:
    logger.error(f"❌ Import error: {e}")
    import traceback
    logger.error(traceback.format_exc())
    sys.exit(1)


class DebugFrankaServer(FrankaServer):
    """Enhanced FrankaServer with detailed logging."""
    
    def __init__(self, cfg):
        logger.info("Initializing DebugFrankaServer...")
        try:
            super().__init__(cfg)
            logger.info("✅ FrankaServer initialized successfully")
        except Exception as e:
            logger.error(f"❌ Error initializing FrankaServer: {e}")
            raise
    
    def control_daemon(self):
        """Override control_daemon with enhanced logging."""
        notify_component_start(component_name="Franka Control Subscriber (Debug)")
        logger.info("Control daemon started, waiting for commands...")
        
        command_count = 0
        last_log_time = time.time()
        
        try:
            while True:
                try:
                    # Log heartbeat every 10 seconds
                    current_time = time.time()
                    if current_time - last_log_time > 10:
                        logger.info(f"💓 Heartbeat: Processed {command_count} commands so far")
                        last_log_time = current_time
                    
                    # Receive command
                    command = self.action_socket.recv()
                    command_count += 1
                    
                    if command == b"get_state":
                        logger.debug(f"[{command_count}] Received get_state request")
                        state = self.get_state()
                        self.action_socket.send(state)
                        
                    else:
                        # Movement command
                        try:
                            import pickle
                            franka_control = pickle.loads(command)
                            
                            logger.info(f"[{command_count}] 📥 RECEIVED MOVEMENT COMMAND:")
                            logger.info(f"   Position: {franka_control.pos}")
                            logger.info(f"   Quaternion: {franka_control.quat}")
                            logger.info(f"   Gripper: {franka_control.gripper}")
                            logger.info(f"   Reset: {franka_control.reset}")
                            logger.info(f"   Timestamp: {franka_control.timestamp}")
                            
                            # Get current state before movement
                            current_quat, current_pos = self._robot.last_eef_quat_and_pos
                            if current_quat is not None and current_pos is not None:
                                logger.info(f"   Current position: {current_pos.flatten()}")
                                expected_movement = np.linalg.norm(franka_control.pos - current_pos.flatten())
                                logger.info(f"   Expected movement: {expected_movement*1000:.2f}mm")
                            
                            # Execute command
                            if franka_control.reset:
                                logger.info("   🔄 Executing RESET command...")
                                self._robot.reset_joints(gripper_open=franka_control.gripper)
                                time.sleep(1)
                                logger.info("   ✅ Reset complete")
                            else:
                                logger.info("   🎯 Executing MOVE command...")
                                self._robot.osc_move(
                                    franka_control.pos,
                                    franka_control.quat,
                                    franka_control.gripper,
                                )
                                logger.info("   ✅ Move command sent to robot")
                            
                            # Send response
                            response_state = self.get_state()
                            self.action_socket.send(response_state)
                            
                            # Log result
                            if response_state != b"state_error":
                                new_state = pickle.loads(response_state)
                                if current_pos is not None:
                                    actual_movement = np.linalg.norm(new_state.pos - current_pos.flatten())
                                    logger.info(f"   📏 Actual movement: {actual_movement*1000:.2f}mm")
                            
                        except Exception as e:
                            logger.error(f"❌ Error processing movement command: {e}")
                            import traceback
                            logger.error(traceback.format_exc())
                            self.action_socket.send(b"state_error")
                            
                except Exception as e:
                    logger.error(f"❌ Error in control loop: {e}")
                    import traceback
                    logger.error(traceback.format_exc())
                    
        except KeyboardInterrupt:
            logger.info("Keyboard interrupt received, shutting down...")
        finally:
            logger.info(f"Control daemon ending. Total commands processed: {command_count}")
            self._robot.close()
            self.action_socket.close()


@hydra.main(version_base="1.2", config_path="configs", config_name="franka_server")
def main(cfg):
    logger.info("Hydra configuration loaded")
    logger.info(f"Deoxys config path: {cfg.deoxys_config_path}")
    
    try:
        fs = DebugFrankaServer(cfg.deoxys_config_path)
        logger.info("Starting server...")
        fs.init_server()
    except Exception as e:
        logger.error(f"❌ Fatal error: {e}")
        import traceback
        logger.error(traceback.format_exc())
        sys.exit(1)


if __name__ == "__main__":
    main() 