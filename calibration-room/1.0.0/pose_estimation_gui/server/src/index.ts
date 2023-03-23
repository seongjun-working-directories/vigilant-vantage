import express from 'express';
import cors from 'cors';
import { commands_router, topics_router, utilities_router } from './router';

const app = express();
const PORT = 5000;

app.use(cors());
app.use(express.json({ limit: 5000000 }));

app.use(commands_router, topics_router, utilities_router);

app.listen(PORT, () => {
  console.log(`Server Listening On Port ${PORT}`);
});