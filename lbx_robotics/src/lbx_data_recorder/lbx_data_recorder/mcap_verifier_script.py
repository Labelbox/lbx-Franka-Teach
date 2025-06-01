#!/usr/bin/env python3
"""
MCAP Data Verifier for Labelbox Robotics System
Verifies the integrity and completeness of recorded MCAP files.
This script can be run from the command line.
"""

import json
import time
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import numpy as np
import argparse # For command-line argument parsing
import sys # For exit codes

import mcap
from mcap.reader import make_reader

class MCAPVerifier:
    """Verifies MCAP recordings for data integrity and completeness"""
    
    def __init__(self, filepath: str, logger_instance=None):
        self.filepath = Path(filepath)
        if not self.filepath.exists():
            raise FileNotFoundError(f"MCAP file not found: {filepath}")
        
        self.logger = logger_instance if logger_instance else self._get_default_logger()
            
        self.verification_results = {
            "file_info": {},
            "data_streams": {},
            "camera_streams": {},
            "errors": [],
            "warnings": [],
            "summary": {}
        }

    def _get_default_logger(self):
        import logging
        logger = logging.getLogger(f"MCAPVerifier_{self.filepath.name}")
        if not logger.hasHandlers(): # Avoid adding multiple handlers if called multiple times
            handler = logging.StreamHandler(sys.stdout)
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            handler.setFormatter(formatter)
            logger.addHandler(handler)
            logger.setLevel(logging.INFO)
        return logger
        
    def verify(self, verbose: bool = True) -> Dict:
        if verbose: self.logger.info(f"\n🔍 Verifying MCAP file: {self.filepath.name}\n============================================================")
        self._verify_file_info()
        self._verify_data_streams(verbose)
        self._generate_summary()
        if verbose: self._print_results()
        return self.verification_results
    
    def _verify_file_info(self):
        stat = self.filepath.stat()
        self.verification_results["file_info"] = {
            "filename": self.filepath.name, "size_mb": stat.st_size / (1024*1024),
            "created": time.ctime(stat.st_ctime), "modified": time.ctime(stat.st_mtime)
        }
        
    def _verify_data_streams(self, verbose: bool):
        with open(self.filepath, "rb") as f:
            reader = make_reader(f)
            summary = reader.get_summary()
            if summary is None: self.verification_results["errors"].append("Failed to read MCAP summary"); return
            
            duration_ns = summary.statistics.message_end_time - summary.statistics.message_start_time
            self.verification_results["file_info"]["duration_sec"] = duration_ns / 1e9
            self.verification_results["file_info"]["duration_str"] = f"{int((duration_ns/1e9)//60):02d}m{int((duration_ns/1e9)%60):02d}s"
            
            topic_data = {}
            for schema, channel, message in reader.iter_messages():
                topic = channel.topic
                if topic not in topic_data:
                    topic_data[topic] = {"schema": schema.name if schema else "?", "encoding": schema.encoding if schema else "?", 
                                         "timestamps": [], "message_count": 0}
                topic_data[topic]["timestamps"].append(message.log_time / 1e9)
                topic_data[topic]["message_count"] += 1
                # Basic content checks can be added here per topic/schema if needed
            
            for topic, data in topic_data.items():
                count = data["message_count"]
                avg_freq, std_freq, duration = 0,0,0
                if len(data["timestamps"]) > 1:
                    ts_arr = np.array(data["timestamps"])
                    duration = ts_arr[-1] - ts_arr[0]
                    if duration > 0: avg_freq = (len(ts_arr)-1) / duration
                    if len(ts_arr) > 2 and duration > 0: std_freq = np.std(1.0 / np.diff(ts_arr))
                
                stream_info = {"schema": data["schema"], "encoding": data["encoding"], "message_count": count,
                               "average_frequency_hz": round(avg_freq,2), "frequency_std_hz": round(std_freq,2),
                               "duration_sec": round(duration,2)}
                if topic.startswith("/camera/"): self.verification_results["camera_streams"][topic] = stream_info
                else: self.verification_results["data_streams"][topic] = stream_info
                
    def _generate_summary(self):
        total_msg = sum(s["message_count"] for s in self.verification_results["data_streams"].values()) + \
                      sum(s["message_count"] for s in self.verification_results["camera_streams"].values())
        self.verification_results["summary"] = {
            "is_valid": len(self.verification_results["errors"]) == 0 and total_msg > 0,
            "total_messages": total_msg,
            "error_count": len(self.verification_results["errors"]),
            "warning_count": len(self.verification_results["warnings"])
        }
        
    def _print_results(self):
        res = self.verification_results
        self.logger.info(f"\n📁 File Info: {res['file_info']['filename']} ({res['file_info']['size_mb']:.2f} MB, {res['file_info']['duration_str']})")
        self.logger.info("\n📊 Data Streams:")
        for topic, info in res["data_streams"].items(): self.logger.info(f"  {topic}: {info['message_count']} msgs, ~{info['average_frequency_hz']:.1f} Hz, Schema: {info['schema']}")
        if res["camera_streams"]: self.logger.info("\n📷 Camera Streams:")
        for topic, info in res["camera_streams"].items(): self.logger.info(f"  {topic}: {info['message_count']} msgs, ~{info['average_frequency_hz']:.1f} Hz, Schema: {info['schema']}")
        if res["errors"]: self.logger.error(f"\n❌ Errors ({len(res['errors'])}):\n  " + "\n  ".join(res["errors"]))
        if res["warnings"]: self.logger.warning(f"\n⚠️ Warnings ({len(res['warnings'])}):\n  " + "\n  ".join(res["warnings"]))
        self.logger.info(f"\n📋 Summary: Valid: {res['summary']['is_valid']}, Total Messages: {res['summary']['total_messages']}")
        self.logger.info("============================================================")

def main():
    parser = argparse.ArgumentParser(description="Verify MCAP data recordings.")
    parser.add_argument("filepath", type=str, help="Path to the MCAP file to verify.")
    parser.add_argument("--verbose", action="store_true", help="Print detailed verification output.")
    args = parser.parse_args()

    # Setup basic logger for the script
    script_logger = logging.getLogger("MCAPVerifierScript")
    script_logger.setLevel(logging.INFO if args.verbose else logging.WARN)
    handler = logging.StreamHandler(sys.stdout)
    formatter = logging.Formatter('%(levelname)s: %(message)s') # Simpler for CLI
    handler.setFormatter(formatter)
    if not script_logger.hasHandlers(): script_logger.addHandler(handler)
    
    try:
        verifier = MCAPVerifier(args.filepath, logger_instance=script_logger)
        results = verifier.verify(verbose=args.verbose)
        if not results["summary"]["is_valid"]:
            script_logger.error("MCAP file verification failed.")
            sys.exit(1)
        else:
            script_logger.info("MCAP file verification successful.")
            sys.exit(0)
    except FileNotFoundError as e:
        script_logger.error(str(e))
        sys.exit(2)
    except Exception as e:
        script_logger.error(f"An unexpected error occurred: {e}", exc_info=args.verbose)
        sys.exit(3)

if __name__ == "__main__":
    main() 