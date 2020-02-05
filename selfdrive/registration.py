import os
import json

from datetime import datetime, timedelta
from selfdrive.swaglog import cloudlog
from selfdrive.version import version, terms_version, training_version, get_git_commit, get_git_branch, get_git_remote
from common.android import get_imei, get_serial, get_subscriber_info, ANDROID
from common.basedir import BASEDIR
from common.api import api_get
from common.params import Params
from common.file_helpers import mkdirs_exists_ok

if ANDROID:
  persist_base = "persist/comma"
else:
  persist_base = os.path.join(BASEDIR, "persist/comma")

def register():
  params = Params()
  params.put("Version", version)
  params.put("TermsVersion", terms_version)
  params.put("TrainingVersion", training_version)
  params.put("GitCommit", get_git_commit())
  params.put("GitBranch", get_git_branch())
  params.put("GitRemote", get_git_remote())
  params.put("SubscriberInfo", get_subscriber_info())

  if not os.path.isfile(persist_base+"id_rsa.pub"):
    cloudlog.warning("generating your personal RSA key")
    mkdirs_exists_ok(persist_base)
    assert os.system("openssl genrsa -out "+persist_base+"/id_rsa.tmp 2048") == 0
    assert os.system("openssl rsa -in "+persist_base+"/id_rsa.tmp -pubout -out "+persist_base+"/id_rsa.tmp.pub") == 0
    os.rename(persist_base+"/id_rsa.tmp", persist_base+"/id_rsa")
    os.rename(persist_base+"/id_rsa.tmp.pub", persist_base+"/id_rsa.pub")

  # make key readable by app users (ai.comma.plus.offroad)
  os.chmod(persist_base+'/', 0o755)
  os.chmod(persist_base+'/id_rsa', 0o744)

  dongle_id, access_token = params.get("DongleId", encoding='utf8'), params.get("AccessToken", encoding='utf8')
  public_key = open(persist_base+"/id_rsa.pub").read()

  # create registration token
  # in the future, this key will make JWTs directly
  private_key = open(persist_base+"/id_rsa").read()

  # late import
  import jwt
  register_token = jwt.encode({'register':True, 'exp': datetime.utcnow() + timedelta(hours=1)}, private_key, algorithm='RS256')

  if ANDROID:
    try:
      cloudlog.info("getting pilotauth")
      resp = api_get("v2/pilotauth/", method='POST', timeout=15,
                     imei=get_imei(), serial=get_serial(), public_key=public_key, register_token=register_token)
      dongleauth = json.loads(resp.text)
      dongle_id, access_token = dongleauth["dongle_id"], dongleauth["access_token"]
      

      params.put("DongleId", dongle_id)
      params.put("AccessToken", access_token)
      return dongle_id, access_token
    except Exception:
      print(dongle_id, access_token)
      cloudlog.exception("failed to authenticate")
      if dongle_id is not None and access_token is not None:
        return dongle_id, access_token
      else:
        return None

  dongle_id = '0375fdf7b1ce594d'
  #access_token = 'MIIEpQIBAAKCAQEAx2Q1WOGH8Zo+EU3Giv8X9OltzRfqMi9b/m6v1lPtSw86vjPSXM0hG4HFTtCr6mA7D9wSu6+HYMVWfcwMQ+d4J36dc0aIsMVZV/rz5o8jORQf8tWe//8dAn5krDPbxxwHZCfHZlZVr/SAI7MI2VZTwZ24rXmE9cZFoTT7CUdl44xBKWKDpIxlPMTGSrbJWjDTytclAHo1Q89x+E85h+mD3LWb7jAE9v1sNB+f9gK3rL0WkzteoaL58Yz8QZhWpP5aMMaHXCtOyJp2SYMyfPRV3OhLPbyq8j71PuxXZ31mRcZG4lUKRfvK3lIGV7ATUCFoucQ9eV7KdYLGGLY2dDgoUwIDAQABAoIBAQCMTSvondHNpK4V+PdC6RrPX82+ahyGMaL21qXuF2I61b2NBuP4HvMz6s2OlvDRW+NTVpTr8ig9ZL5miET9Jwv6JnLeIXyffk5OxSwISSIM0OdmNQDLu5SLI6rtLRRx93wP8pmxngBvbokYFaudWsxWygegMYjodBqcw639Kns8cD2F5yHXbx5Ek5ueIvOH+KTBwgHXfCIpuKZdmR/EgowQAe5qhNNAMqxTxp8etoy0dWE6Ho5OarkZyM5gSWNqFtyImvm+P3YNw+zDhzi/yM4h4VAN6Z9nl744Ap6lylRylC7O7BUPDUF/JiM5sbxMaIUJTkp1UL2feccqtX9qDotpAoGBAP1DdCAIQoG6rPib7QFofF9DU/AOtftrLAe5mqfgg3cSibX2ZQwl4PHrWNSG/6t4PQCgz2aJfhwDJwL9/boL6B00AE5W589QswMuHn59VBkL4ePsFelkrkKMhROUgVIUHnM/ln1itTHMaoPagvr6hWCIcOduY12BP9aVKPGLTckVAoGBAMmLvZJxqmlXb+s0yfA1XRhkPC39y+Cucv7p+mREp7xb3cy6o1Hr12Nqc4UEwtgYOKn4fCpiULbNdndATsmiI7rmeMS9sGncfpZjiHly0wFJIwmqpvnyphPJHs+N/LohKiL91NEeWfPQwBOUZzZJ5EIMuUbSxKagNNe9hr14gbXHAoGBAPIbW5WLj9hoVhZLKORhuBNxT3p69ajlKPAxlBDL5aEcjZzvcGWOiF17uC1i6GJIzTgegmXpJi8tLEwF9lm2LCxOm8RH+84JvTdbDTeqI4zV+RljhQbagmOH5Wd0XbPTG9DchsZSaBi061Yku3Wlq38+r3/t3Pt0JUzCxqMxoiiJAoGARcpz/NALeKrkgIgpHVr9P8MF688k2ErQW2FeaXY0t08AgTJn96g5T52/HuLFLdtFU+ZN6Mdet85yJV1Jax0QTlocm9Qr0Jyf4SfaTGPsmVdgIKrNY2QZCmTsnGbIixOM6c0H8toVwA4bVSxgiTYRKqYJSnMV0bVMVyC0Fcwy7mUCgYEAlxdbqww3QedidLpXcBPZsYO5+ioTVBcCzNon5ZpJqL4xi0gWLNayUwY1mC+vS9e3qz3VPYipiQWjsWa67JyFuUiQPrvD8eCUudfEHKDYk1y193kxGhpnw0YPjlFEtjP2FZgSYowa/OOswy2rcCAZcNMw6lGF5ZbvZuwl6NyZ+J4='
  access_token = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpZGVudGl0eSI6ImZvbyIsImlhdCI6MTU2MDc1NDUzMywibmJmIjoxNTYwNzU0NTMzLCJleHAiOjE1OTIyOTA1MzN9.V34lp0Nq0EZMw7b5I8xnZaMvIbS2eisUsrFbXg4itaY'
  return dongle_id, access_token

if __name__ == "__main__":
  print(api_get("").text)
  print(register())
