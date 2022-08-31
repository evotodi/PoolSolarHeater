Import("env")
import os
import ota_updater as ota

def on_upload(source, target, env):
    config = env.GetProjectConfig()
    firmware_path = str(source[0])
    print(f"FW Path {os.path.join(env['PROJECT_DIR'], firmware_path)}")
    fd = open(os.path.join(os.getcwd(), firmware_path), 'rb')
    fw_buffer = fd.read()
    fd.close()
    firmware = bytearray()
    firmware.extend(fw_buffer)

    mqUser = config.get("settings", "mqUser")
    mqPass = config.get("settings", "mqPass")
    mqBroker = config.get("settings", "mqBroker")

    ota.main(broker_host=mqBroker, broker_port=1883, broker_ca_cert=None, broker_username=mqUser, broker_password=mqPass, base_topic='homie/', device_id='poolheater', firmware=firmware)

# env.AddPostAction("upload", on_upload)
env.Replace(UPLOADCMD=on_upload)