import './styles/app.css';
import './styles/shell.css';
import './ui/desmosKeypad.css';
import { boot } from './app/simulator.js';

boot();

const toy = document.querySelector('script[src*="toy-sdk"]');
if (import.meta.env.PROD && 'serviceWorker' in navigator && !toy) {
  window.addEventListener('load', () => {
    navigator.serviceWorker.register('/sw.js').catch(() => {});
  });
}
