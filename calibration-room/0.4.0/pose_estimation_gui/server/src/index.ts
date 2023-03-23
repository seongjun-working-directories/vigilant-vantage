import express from 'express';
import cors from 'cors';
import { router } from './router';

const app = express();
const PORT = 5000;

app.use(cors());
app.use(express.json({ limit: 5000000 }));

app.use(router);

app.listen(PORT, () => {
  console.log(`Server Listening On Port ${PORT}`);
});