"""Browser QA. No clicks unless a fresh simulation server is verified."""
import json
from pathlib import Path
from urllib.request import urlopen
from playwright.sync_api import sync_playwright

URL='http://127.0.0.1:8766'
with urlopen(URL+'/api/state') as r:
    initial=json.load(r)
assert initial['simulate'] and not initial['connected'], 'Fresh simulation server required'
out=Path(__file__).resolve().parents[1]/'logs'/'qa'
out.mkdir(parents=True,exist_ok=True)
with sync_playwright() as p:
    browser=p.chromium.launch(executable_path='/opt/google/chrome/chrome',headless=True,args=['--no-sandbox','--disable-dev-shm-usage'])
    page=browser.new_page(viewport={'width':1440,'height':920})
    errors,posts=[],[]
    page.on('pageerror',lambda error:errors.append(str(error)))
    page.on('request',lambda req:posts.append(req.url) if req.method=='POST' else None)
    page.goto(URL)
    page.wait_for_selector('.joint')
    page.wait_for_timeout(700)
    assert posts==[], 'Loading must not submit actions'
    assert page.request.get(URL+'/api/state').json()['tx_count']==0
    assert page.locator('.joint').count()==7
    assert page.locator('#execute').count()==0
    page.screenshot(path=str(out/'desktop.png'),full_page=True)
    page.locator('#connect').click()
    page.wait_for_function("() => document.querySelector('#status').textContent==='已连接'")
    first=page.locator('.joint[data-joint="1"]')
    assert first.locator('input').is_disabled()
    assert page.locator('#reset').is_disabled()
    assert page.locator('.joint button').count()==0
    page.locator('#enable-all').click()
    page.wait_for_function("() => !document.querySelector('.joint[data-joint=\"1\"] input').disabled")
    assert all(j['enabled'] for j in page.request.get(URL+'/api/state').json()['joints'])
    assert not any('/api/target' in url for url in posts), 'Enable must not change goal'
    # Programmatic input must not be treated as a manual gesture.
    page.evaluate('''() => {const s=document.querySelector('.joint[data-joint="1"] input');s.value=30;s.dispatchEvent(new Event('input',{bubbles:true}));}''')
    page.wait_for_timeout(350)
    assert not any('/api/target' in url for url in posts)
    # A real mouse drag changes the target without any execute button.
    box=first.locator('input').bounding_box()
    middle=box['x']+box['width']/2
    y=box['y']+box['height']/2
    page.mouse.move(middle,y)
    page.mouse.down()
    page.mouse.move(middle+box['width']*.08,y,steps=8)
    page.wait_for_timeout(350)
    state=page.request.get(URL+'/api/state').json()
    assert state['joints'][0]['target_deg']>10, state['joints'][0]
    assert any('/api/target' in url for url in posts), 'Must send while dragging'
    page.mouse.up()
    page.wait_for_function("() => Number(document.querySelector('.joint[data-joint=\"1\"] .actual span').textContent)>.1")
    assert all(j['actual_deg']==0 for j in state['joints'][1:])
    page.locator('#stop').click()
    page.wait_for_timeout(200)
    stopped=page.request.get(URL+'/api/state').json()['joints'][0]
    assert stopped['enabled'] and not stopped['moving']
    page.wait_for_timeout(300)
    later=page.request.get(URL+'/api/state').json()['joints'][0]
    assert abs(later['actual_deg']-stopped['actual_deg'])<.001
    page.screenshot(path=str(out/'connected-simulation.png'),full_page=True)
    page.locator('#reset').click()
    page.wait_for_function("() => document.querySelector('#reset').textContent==='复位中…'")
    page.wait_for_function("() => document.querySelector('#reset').textContent==='复位到零位'")
    reset=page.request.get(URL+'/api/state').json()
    assert all(abs(j['actual_deg'])<=1 and j['enabled'] for j in reset['joints'])
    assert any('/api/reset' in url for url in posts)
    page.locator('#disable-all').click()
    page.wait_for_function("() => document.querySelector('#enable-all').textContent==='全部使能'")
    assert first.locator('input').is_disabled()
    page.set_viewport_size({'width':390,'height':844})
    page.wait_for_timeout(200)
    assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')
    page.screenshot(path=str(out/'mobile.png'),full_page=True)
    assert errors==[], errors
    page.locator('#connect').click()
    page.wait_for_function("() => document.querySelector('#status').textContent==='未连接'")
    browser.close()
print('Browser QA passed: no startup actions, one-click group enable, trusted live drag, stop clears movement, group disable, responsive layout')
